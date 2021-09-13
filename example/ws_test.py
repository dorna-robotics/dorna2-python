from __future__ import print_function
import sys
import time
import json
sys.path.append('..')
from dorna2 import dorna


def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    # tik
    start = time.time()
    robot = dorna()
    robot.connect(arg["ip"], arg["port"])
    for cmd in 100 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print("arg: ", arg)
        robot.play(**arg)
        robot.complete(arg["id"])

    # tok
    print(time.time()-start)

    # close connection
    robot.close()

    print("connection closed")
if __name__ == '__main__':
    main("config.json")
