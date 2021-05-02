from __future__ import print_function
from random import random
import sys
import json
sys.path.append('..')
from dorna2 import dorna


def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    robot.connect(arg["ip"], arg["port"])

    # start position
    print("going to start ->")
    arg = {"rel": 0, "id": robot.rand_id(), "j0": 1, "j1": 1, "j2": 0, "j3": 0, "j4": 0}
    robot.jmove(**arg)

    # come 200 mm back toward x
    print("go back in x direction")
    arg = {"rel": 1, "id": robot.rand_id(), "x": -200, "vel": 500, "accel": 2000, "jerk": 4000}
    trk = robot.lmove(True, **arg)
    trk.complete()

    # random points
    i = 0
    while True:
        arg = {"rel": 0, "id": i + 1, "x": 300 + 1 * random(), "y": -100 + 1 * random(), "z": 206.404 - 100 + 1 * random()}
        print("command", i, "   arg: ", arg)
        trk = robot.lmove(True, **arg)
        trk.complete()
        i += 1
    robot.close()

if __name__ == '__main__':
    main("config.json")
