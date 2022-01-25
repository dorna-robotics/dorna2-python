from __future__ import print_function
import sys
import time
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

    print("connected")

    # tik
    start = time.time()
    for cmd in 10 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print("send: ", arg)
        print("receive: ", robot.play(**arg))
        print("####")
    # tok
    print("total time: ",time.time()-start)
    print("####")

    # close connection
    robot.close()

    print("connection closed")
if __name__ == '__main__':
    main("config.json")
