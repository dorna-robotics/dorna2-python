from __future__ import print_function
from random import random
import sys
import json
sys.path.append('..')
from dorna2 import dorna


def random_joint():
    j0 = -90 + random() * 180
    j1 = -45 + random() * 135
    j2 = max(-90, -2.7*j1 - 60) + random() * (90 - max(-90, -2.7*j1 - 60))
    j3 = -90 + random() * 180
    j4 = 0
    return j0, j1, j2, j3, j4


def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    robot.connect(arg["ip"], arg["port"])

    # go home
    arg = {"cmd": "jmove", "rel": 0, "id": robot.rand_id(), "j0": 0, "j1": 0, "j2": 0, "j3": 0, "j4": 0, "vel": 50, "accel": 300, "jerk": 1000}
    print("going to start ->")
    robot.play(**arg)

    # random points
    i = 0
    while True:
        j0, j1, j2, j3, j4 = random_joint()

        arg = {"cmd": "jmove", "rel": 0, "id": i+1, "j0": j0, "j1": j1, "j2": j2, "j3": j3, "j4": j4}
        print("command", i, "   arg: ", arg)
        trk = robot.play(True, **arg)
        trk.complete()
        i += 1

    robot.close()

if __name__ == '__main__':
    main("config.json")
