from __future__ import print_function
from random import random
import sys
sys.path.append('..')
from dorna2 import dorna


def main(ip, port):
    robot = dorna()
    robot.connect(ip, port)

    # go home
    arg = {"cmd": "rmove", "rel": 0, "id": robot.rand_id(), "j0": 0, "j1": 0, "j2": 0, "j3": 0, "j4": 0, "vel": 0.5, "accel": 0.3, "jerk": 0.1}
    print("going home ->")
    robot.play(**arg)

    # random points
    i = 0
    while True:
        j0 = -90 + random() * 180
        j1 = -45 + random() * 135
        j2 = max(-90, -2.7*j1 - 60) + random() * (90 - max(-90, -2.7*j1 - 60))
        j3 = -90 + random() * 180
        j4 = 0

        arg = {"cmd": "rmove", "rel": 0, "id": i+1, "j0": j0, "j1": j1, "j2": j2, "j3": j3, "j4": j4}
        print("command", i, "   arg: ", arg)
        robot.play(**arg)
        robot.wait(id=arg["id"], stat=2)
        i += 1

    robot.ws.close()

if __name__ == '__main__':
    main("127.0.0.1", 443)
