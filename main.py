# Importamos las clases que se requieren para manejar los agentes (Agent) y su entorno (Model).
# Cada modelo puede contener múltiples agentes.
from mesa import Agent, Model 

# Debido a que necesitamos que existe un solo agente por celda, elegimos ''SingleGrid''.
from mesa.space import SingleGrid

# Con ''RandomActivation'', hacemos que todos los agentes se activen ''al mismo tiempo''.
from mesa.time import RandomActivation
from mesa.time import BaseScheduler

# Haremos uso de ''DataCollector'' para obtener información de cada paso de la simulación.
from mesa.datacollection import DataCollector

# matplotlib lo usaremos crear una animación de cada uno de los pasos del modelo.
#%matplotlib inline
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
plt.rcParams["animation.html"] = "jshtml"
matplotlib.rcParams['animation.embed_limit'] = 2**128

# Importamos los siguientes paquetes para el mejor manejo de valores numéricos.
import numpy as np
import pandas as pd

# Definimos otros paquetes que vamos a usar para medir el tiempo de ejecución de nuestro algoritmo.
import time
import datetime

from random import randrange
import math

#Config
M = 40
N = 30
K = 25
ROBOT_NUM = 5
BOX_NUM = K
SHELF_NUM = math.ceil(BOX_NUM / 5)
MAP_W = M
MAP_H = N
MAX_STACK = 5

#Const
DIR_UP = 0
DIR_DOWN = 2
DIR_LEFT = 3
DIR_RIGHT = 1

LOC_EMPTY = 0
LOC_ROBO = 1
LOC_BOX = 2
LOC_SHELF = 3

COLOR_EMPTY = 0
COLOR_ROBO = 10
COLOR_ROBO_BOX = 10
COLOR_BOX = 50
COLOR_SHELF = 250
COLOR_SHELF_BOX = 1

MAX_GENERATIONS = 100




def get_grid(model):
    grid = np.zeros((model.grid.width, model.grid.height))
    for (content, x, y) in model.grid.coord_iter():
        if(content == None):
            grid[x][y] = 0
        else:
            if(content.loc_type == LOC_ROBO):
                grid[x][y] = content.box * COLOR_ROBO_BOX + COLOR_ROBO
            elif(content.loc_type == LOC_BOX):
                grid[x][y] = content.box * COLOR_BOX
            elif(content.loc_type == LOC_SHELF):
                grid[x][y] = content.box * COLOR_SHELF_BOX + COLOR_SHELF
        
    return grid

class CellAgent(Agent):
    def __init__(self, unique_id, model, loc_type):
        super().__init__(unique_id, model)

        self.loc_type = loc_type

        if(self.loc_type == LOC_BOX):
            self.box = 1
        else:
            self.box = 0

        #property of robo
        self.uDir = DIR_UP
        self.bCarryBox = False

    def getDirTo(self, pos):
        result = None
        if(self.pos[0] == pos[0]):
            if(self.pos[1] > self.pos[1]):
                result = DIR_DOWN
            else:
                result = DIR_UP
        elif(self.pos[1] == pos[1]):
            if(self.pos[0] > self.pos[0]):
                result = DIR_LEFT
            else:
                result = DIR_RIGHT
        return result

    def routeTo(self, target, visited, dir, distance):
        result = None
        neighbors = self.model.grid.get_neighbors(self.pos, moore = True, include_center = False)
        for neighbor in neighbors:
            if(neighbor.loc_type == target and neighbor.box < MAX_STACK):#found
                result = (dir, distance)
                break
            elif(neighbor.loc_type == LOC_EMPTY):
                if(visited[self.model.grid.width * self.pos[1] + self.pos[0]] == False):
                    if(dir == None):
                        my_dir = self.getDirTo(neighbor.pos)
                    else:
                        my_dir = dir

                    visited_copy = visited.copy()
                    visited_copy[self.model.grid.width * self.pos[1] + self.pos[0]] = True
                    neighbor.routeTo(target, visited_copy, my_dir, distance + 1)
        return result

    def move(self, dir):
        if(dir == DIR_UP):
            pos = (self.pos[0], self.pos[1] + 1)
        elif(dir == DIR_DOWN):
            pos = (self.pos[0], self.pos[1] - 1)
        elif(dir == DIR_LEFT):
            pos = (self.pos[0] - 1, self.pos[1])
        elif(dir == DIR_RIGHT):
            pos = (self.pos[0] + 1, self.pos[1])

        self.model.grid.move_agent(self, pos)
        self.uDir = dir

    def findDir(self):
        result = None
        for count in range(4):
            dir = self.uDir

            if(dir == DIR_UP):
                pos = (self.pos[0], self.pos[1] + 1)
            elif(dir == DIR_DOWN):
                pos = (self.pos[0], self.pos[1] - 1)
            elif(dir == DIR_LEFT):
                pos = (self.pos[0] - 1, self.pos[1])
            elif(dir == DIR_RIGHT):
                pos = (self.pos[0] + 1, self.pos[1])
            
            if(self.model.grid.is_cell_empty(pos) and (not self.model.grid.out_of_bounds(pos))):
                result = dir
                break
            
            dir += 1
            if(dir >= 4):
                dir = 0

        return result

    def pick(self, cell):
        if(self.box == 0 and self.bCarryBox == False):
            if(cell.loc_type == LOC_BOX and cell.box > 0):
                cell.box -= 1
                self.box = 1
                self.bCarryBox = True

                if(cell.box <= 0):
                    cell.model.grid.remove_agent(cell)
            elif(cell.loc_type == LOC_BOX and cell.box == 0):
                cell.model.grid.remove_agent(cell)
            else:
                pass
        else:
            if(self.box == 0):
                self.bCarryBox = False
            else:
                self.bCarryBox = True

    def put(self, cell):
        if(self.box == 1 and self.bCarryBox == True):
            if((cell.loc_type == LOC_BOX or cell.loc_type == LOC_SHELF) and cell.box < MAX_STACK):
                cell.box += 1
                self.box = 0
                self.bCarryBox = False
            else:
                pass
        else:
            if(self.box == 0):
                self.bCarryBox = False
            else:
                self.box == 1
                self.bCarryBox = True


    def step(self):
        if(self.loc_type == LOC_ROBO):            
            move_done = False
            if(self.bCarryBox):#carry box
                neighbors = self.model.grid.get_neighbors(self.pos, moore = True, include_center = False)
                for neighbor in neighbors:
                    if(neighbor.loc_type == LOC_SHELF):
                        self.put(neighbor)
                        move_done = True
                if(move_done == False):
                    if(len(neighbors) < 8):
                        dir = self.findDir()
                        if(dir != None):
                            self.move(dir)
                        else:#stack there, put box to max stack
                            neighbor_box_max = None
                            for neighbor in neighbors:
                                if(neighbor.loc_type == LOC_BOX and neighbor.box < MAX_STACK):
                                    if(neighbor_box_max == None):
                                        neighbor_box_max = neighbor
                                    else:
                                        if(neighbor_box_max.box < neighbor.box):
                                            neighbor_box_max = neighbor
                            if(neighbor_box_max != None):
                                self.put(neighbor_box_max)
                                move_done = True
                            else:#really stuck there, surround by boxes
                                pass                                

            else:#without box
                neighbors = self.model.grid.get_neighbors(self.pos, moore = True, include_center = False)
                neighbor_box_min = None
                for neighbor in neighbors:#pick up the stack with min box
                    if(neighbor.loc_type == LOC_BOX):
                        if(neighbor_box_min == None):
                            neighbor_box_min = neighbor
                        else:
                            if(neighbor_box_min.box > neighbor.box):
                                neighbor_box_min = neighbor
                if(neighbor_box_min != None):
                    self.pick(neighbor_box_min)
                    move_done = True
                else:
                    dir = self.findDir()
                    if(dir != None):
                        self.move(dir)
                        move_done = True
                        

        

        
        
            

class WarehouseModel(Model):
    def __init__(self, width, height, num_robo, num_box):
        self.num_robo = num_robo
        self.num_box = num_box
        self.num_shelf = math.ceil(num_box / num_robo)
        self.grid = SingleGrid(width, height, False)
        self.schedule = BaseScheduler(self)

        self.datacollector = DataCollector(model_reporters={"Grid": get_grid})

        remain_grid = list(range(width * height))
    
        #assign robo location
        #print("w " + str(width) + ", h " + str(height))
        for count in range(num_robo):
            index = randrange(len(remain_grid))
            value = remain_grid[index]
            remain_grid.pop(index)
            #y = value // width
            #x = value % width 

            (x, y) = self.grid.find_empty()
            #print(str(x) + ", " + str(y))
            cell = CellAgent(value, self, LOC_ROBO)
            self.grid.place_agent(cell, (x, y))
            self.schedule.add(cell)

        #assign box location
        for count in range(num_box):
            index = randrange(len(remain_grid))
            value = remain_grid[index]
            remain_grid.pop(index)
            #x = value // width
            #y = value % width 
            (x, y) = self.grid.find_empty()
            cell = CellAgent(value, self, LOC_BOX)
            self.grid.place_agent(cell, (x, y))
            self.schedule.add(cell)

        #assign shelf location
        for count in range(self.num_shelf):
            index = randrange(len(remain_grid))
            value = remain_grid[index]
            remain_grid.pop(index)
            #x = value // width
            #y = value % width 
            (x, y) = self.grid.find_empty()
            cell = CellAgent(value, self, LOC_SHELF)
            self.grid.place_agent(cell, (x, y))
            self.schedule.add(cell)            

        if(False):
            #assign empty location
            for count in range(len(remain_grid)):
                index = randrange(len(remain_grid))
                value = remain_grid[index]
                remain_grid.pop(index)
                x = value // width
                y = value % width 
                cell = CellAgent(value, self, LOC_EMPTY)
                self.grid.place_agent(cell, (x, y))
                self.schedule.add(cell)
    
    def isDone(self):
        result = True

        for (content, x, y) in self.grid.coord_iter():
            if(content.loc_type == LOC_ROBO):
                result = False
                break
        return result

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()

if __name__ == "__main__":
    step_count = 0
    start_time = time.time()
    model = WarehouseModel(MAP_W, MAP_H, ROBOT_NUM, BOX_NUM)

    for i in range(MAX_GENERATIONS):
        step_count += 1
        model.step()
        if(model.isDone):
            break

    print("Time Spend: " + str(datetime.timedelta(seconds=(time.time() - start_time))))
    print("Step Execute: " + str(step_count))

    all_grid = model.datacollector.get_model_vars_dataframe()

    fig, axs = plt.subplots(figsize=(MAP_W, MAP_H))
    axs.set_xticks([])
    axs.set_yticks([])
    patch = plt.imshow(all_grid.iloc[0][0], cmap=plt.cm.binary)

    def animate(i):
        patch.set_data(all_grid.iloc[i][0])

    anim = animation.FuncAnimation(fig, animate, frames=step_count)
    plt.show()

