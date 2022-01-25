from __future__ import print_function
import sys
import json
from dorna2 import dorna

def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    print("connecting")
    if not robot.connect(arg["ip"], arg["port"]):
        print("not connected")
        robot.close()
        return 0

    # use probe to wait for input0 turns 1
    robot.probe(in0=1)
    robot.close()

if __name__ == '__main__':
    main("config.json")
