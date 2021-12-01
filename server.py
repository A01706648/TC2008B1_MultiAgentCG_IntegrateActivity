from http.server import BaseHTTPRequestHandler, HTTPServer
import logging
import json

import numpy as np
from vector import Vector
from main import MAP_H, ROBOT_NUM, WarehouseModel

# Set the number of agents here:
#flock = [Boid(*np.random.rand(2)*30, width, height) for _ in range(20)]
model = None

class cPos():
    def __init__(self, x, y, z=0):
        self.x = x
        self.y = y
        self.z = z

def updatePositions():
    global model
    positions = []

    if(model != None):
        model.step()

        robo_list = model.getAllRobot()
        box_list = model.getAllBox()
        shelf_list = model.getAllShelf()

        for robo in robo_list:
            #print(robo.pos)
            positions.append(Vector(robo.pos[0], robo.pos[1]))

        for box in box_list:
            if(box.pos == None):
                positions.append(Vector(box.onStackPos[0], box.onStackPos[1], box.stackIndex))
            else:
                positions.append(Vector(box.pos[0], box.pos[1]))

        for shelf in shelf_list:
            positions.append(Vector(shelf.pos[0], shelf.pos[1]))
        
        #print(positions)
    #for boid in flock:
    #    boid.apply_behaviour(flock)
    #    boid.update()
    #    pos = boid.edges()
    #    positions.append(pos)
    return positions

def positionsToJSON(ps):
    posDICT = []
    for p in ps:
        pos = {
            "x" : p[0],
            "z" : p[1],
            "y" : p[2]
        }
        posDICT.append(pos)
    return json.dumps(posDICT)

class Server(BaseHTTPRequestHandler):
    
    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        
    def do_GET(self):
        logging.info("GET request,\nPath: %s\nHeaders:\n%s\n", str(self.path), str(self.headers))
        self._set_response()
        self.wfile.write("GET request for {}".format(self.path).encode('utf-8'))

    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        #post_data = self.rfile.read(content_length)
        post_data = json.loads(self.rfile.read(content_length))
        #logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                     #str(self.path), str(self.headers), post_data.decode('utf-8'))
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                     str(self.path), str(self.headers), json.dumps(post_data))
        

        #########################My Code        
        global model
        
        if(model == None):
            MAP_W = int(post_data["MAP_W"])
            MAP_H = int(post_data["MAP_H"])
            ROBOT_NUM = int(post_data["ROBOT_NUM"])
            BOX_NUM = int(post_data["BOX_NUM"])

            model = WarehouseModel(MAP_W, MAP_H, ROBOT_NUM, BOX_NUM)

            print(MAP_W)
            print(MAP_H)
            print(ROBOT_NUM)
            print(BOX_NUM)
        #########################
        '''
        x = post_data['x'] * 2
        y = post_data['y'] * 2
        z = post_data['z'] * 2
        
        position = {
            "x" : x,
            "y" : y,
            "z" : z
        }

        self._set_response()
        #self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))
        self.wfile.write(str(position).encode('utf-8'))
        '''
        
        positions = updatePositions()
        #print(positions)
        self._set_response()
        resp = "{\"data\":" + positionsToJSON(positions) + "}"
        #print(resp)
        self.wfile.write(resp.encode('utf-8'))

def run(server_class=HTTPServer, handler_class=Server, port=8585):
    logging.basicConfig(level=logging.INFO)
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    logging.info("Starting httpd...\n") # HTTPD is HTTP Daemon!
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:   # CTRL+C stops the server
        pass
    httpd.server_close()
    logging.info("Stopping httpd...\n")

if __name__ == '__main__':
    from sys import argv
    
    
    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()    