from __future__ import print_function
import re
from random import random
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
import json
import sys
sys.path.append('..')
from dorna2 import dorna



class svg(object):
    """svg for ClassName"""
    def __init__(self, t=10):
        super(svg, self).__init__()
        self.t = t

    def parse_text(self, s):
        res = re.split('([-+]?\d+\.\d+)|([-+]?\d+)', s.strip())
        res_f = [r.strip() for r in res if r is not None and r.strip() != '']
        rtn = []
        for s in res_f:
            if s != ",":
                try:
                    rtn.append(float(s))
                except:
                    rtn += [x for x in s]
        return rtn        

    # this function generates the paths for a filename and plots the results
    def gen(self, file_name, x_max, y_max, x_min, y_min):
        paths = []
        path_x = []
        path_y = []

        # first we read the svg file
        tree = ET.parse(file_name)
        root = tree.getroot()
        for path in root.iter('{http://www.w3.org/2000/svg}path'):

            d = self.parse_text(path.attrib['d'])

            cur_x = 0
            cur_y = 0

            start_x = 0
            start_y = 0

            current_cmd = None

            i = 0
            while i < len(d):
                element = d[i]
                if type(element) == str:
                    current_cmd = element
                    i += 1
                # when there is an m, a new path has started
                if current_cmd == 'm':

                    if path_x:
                        paths.append([path_x, path_y])
                        path_x = []
                        path_y = []
                        # we should start a new path here
                        # we should read x and y as the first points
                    cur_x += d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y += d[i]
                    path_y.append(cur_y)
                    i += 1

                    start_x = cur_x
                    start_y = cur_y


                elif current_cmd == 'M':

                    if path_x:
                        paths.append([path_x, path_y])
                        path_x = []
                        path_y = []
                        # we should start a new path here
                        # we should read x and y as the first points
                    cur_x = d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y = d[i]
                    path_y.append(cur_y)
                    i += 1

                    start_x = cur_x
                    start_y = cur_y


                elif current_cmd == 'v':

                    cur_y += d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'V':

                    cur_y = d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'h':

                    cur_x += d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'H':

                    cur_x = d[i]
                    path_x.append(cur_x)
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'l':
                    cur_x += d[i]
                    path_x.append(cur_x)
                    i += 1

                    cur_y += d[i]
                    path_y.append(cur_y)
                    i += 1

                elif current_cmd == 'z' or current_cmd == 'Z':
                    path_x.append(start_x)
                    path_y.append(start_y)
                    cur_x = start_x
                    cur_y = start_y

                elif current_cmd == 'q':
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 
                        p_x = (1 - t) ** 2 * (p0_x +cur_x) + 2 * t * (1 - t) * (p1_x+cur_x) + t ** 2 * (p2_x+cur_x)
                        p_y = (1 - t) ** 2 * (p0_y+cur_y) + 2 * t * (1 - t) * (p1_y+cur_y) + t ** 2 * (p2_y+cur_y)
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x += p2_x
                    cur_y += p2_y

                elif current_cmd == 'Q':
                    p0_x = cur_x
                    p0_y = cur_y

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 
                        p_x = (1 - t) ** 2 * p0_x + 2 * t * (1 - t) * p1_x + t ** 2 * p2_x
                        p_y = (1 - t) ** 2 * p0_y + 2 * t * (1 - t) * p1_y + t ** 2 * p2_y
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x = p2_x
                    cur_y = p2_y

                elif current_cmd == 's':
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    p2_x = p1_x
                    i += 1

                    p1_y = d[i]
                    p2_y = p1_y
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 

                        p_x = (1 - t) ** 3 * p0_x + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * p0_y + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(cur_x + p_x)
                        path_y.append(cur_y + p_y)

                    cur_x += p3_x
                    cur_y += p3_y

                elif current_cmd == 'S':
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    p2_x = p1_x
                    i += 1

                    p1_y = d[i]
                    p2_y = p1_y
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 

                        p_x = (1 - t) ** 3 * p0_x + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * p0_y + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(cur_x + p_x)
                        path_y.append(cur_y + p_y)

                    cur_x = p3_x
                    cur_y = p3_y

                elif current_cmd == 'c':
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 
                        p_x = (1 - t) ** 3 * (p0_x + cur_x) + 3 * t * (1 - t) ** 2 * (p1_x + cur_x) + 3 * (t**2) * (1 - t) * (p2_x + cur_x) + t ** 3 * (p3_x+ cur_x)
                        p_y = (1 - t) ** 3 * (p0_y + cur_y) + 3 * t * (1 - t) ** 2 * (p1_y + cur_y) + 3 * (t**2) * (1 - t) * (p2_y + cur_y) + t ** 3 * (p3_y+ cur_y)
                        
                        path_x.append(p_x)
                        path_y.append(p_y)

                        #path_x.append(cur_x + p_x)
                        #path_y.append(cur_y + p_y)


                    cur_x += p3_x
                    cur_y += p3_y

                elif current_cmd == 'C':
                    p0_x = 0
                    p0_y = 0

                    p1_x = d[i]
                    i += 1

                    p1_y = d[i]
                    i += 1

                    p2_x = d[i]
                    i += 1

                    p2_y = d[i]
                    i += 1

                    p3_x = d[i]
                    i += 1

                    p3_y = d[i]
                    i += 1

                    for j in range(self.t):
                        t = (j+1)/self.t 
                        p_x = (1 - t) ** 3 * (p0_x + cur_x) + 3 * t * (1 - t) ** 2 * p1_x + 3 * (t**2) * (1 - t) * p2_x + t ** 3 * p3_x
                        p_y = (1 - t) ** 3 * (p0_y + cur_y) + 3 * t * (1 - t) ** 2 * p1_y + 3 * (t**2) * (1 - t) * p2_y + t ** 3 * p3_y
                        path_x.append(p_x)
                        path_y.append(p_y)

                    cur_x = p3_x
                    cur_y = p3_y



        paths.append([path_x, path_y])

        # now we should scale the paths
        # we first find the min and max of the paths

        x_path_min = min([min(pairs[0]) for pairs in paths])
        x_path_max = max([max(pairs[0]) for pairs in paths])

        y_path_min = min([min(pairs[1]) for pairs in paths])
        y_path_max = max([max(pairs[1]) for pairs in paths])

        # next we scale the pairs
        paths = [[[(x_max - x_min) * (x- x_path_min) / (x_path_max - x_path_min) + x_min for x in pairs[0]], [(y_max - y_min) * (y - y_path_min) / (y_path_max - y_path_min) + y_min for y in pairs[1]]] for pairs in paths]
        for path_pair in paths:
            plt.plot(path_pair[0], path_pair[1])

        plt.show()
        return self.gen_cmd(paths)


    def gen_cmd(self, paths):
        cmds = []
        j = 100
        cmds.append([{"cmd": "lmove", "rel": 1, "z": 3, "vel":150,"accel":200,"jerk":1000, "id":j}])
        for path_pair in paths:
            cmd = []
            # go down
            last_point = list([path_pair[0][0], path_pair[1][0]])
            cmd.append({"cmd": "lmove", "rel": 0, "x": path_pair[0][0], "y": path_pair[1][0]})
            cmd.append({"cmd": "lmove", "rel": 1, "z": -3})
            for i in range(len(path_pair[0])):
                #if (path_pair[0][i] - last_point[0])**2 + (path_pair[1][i] - last_point[1])**2 > 0.01:
                cmd.append({"cmd": "lmove", "rel": 0, "x": path_pair[0][i], "y": path_pair[1][i], "cont":0, "corner": 1})
                last_point = last_point = [path_pair[0][i], path_pair[1][i]]
                
            last_cmd = cmd.pop()
            if "cont" in last_cmd:
                last_cmd["cont"] = 0
            cmd.append(last_cmd)
            j += 100
            cmd.append({"cmd": "lmove", "rel": 1, "z": 3, "id":j})
            cmds.append(cmd)
        return cmds




if __name__ == '__main__':
    
    # now lets write the same thing  and delete it:
    x_max = -30
    x_min = -160
    y_max = 440
    y_min = 300


    cmds = svg(10).gen('cat.svg', x_max, y_max, x_min, y_min)


    robot = dorna()
    print("connecting")
    robot.connect("192.168.254.35", 443)
    wait_id = 100
    for cmd in cmds:
        print("len: ",len(cmd))
        for c in cmd:
            msg = json.dumps(c)
            print(msg)
            robot.play(msg)
        print("wait")
        robot.wait(id=wait_id, stat=2)
        wait_id += 100
    
    robot.close()
