from __future__ import print_function
import sys
import time
import json
sys.path.append('..')
from dorna2 import dorna
from sys import stdout

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

    print("connected")

    # tik
    start = time.time()
    for cmd in 10 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print("arg: ", arg, flush=True)
        robot.play(**arg)
        robot.complete(arg["id"])

    # tok
    print(time.time()-start)

    # close connection
    robot.close()

    print("connection closed")
if __name__ == '__main__':
    main("config.json")
