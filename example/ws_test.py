import time
import json
from dorna2 import dorna

def main(config_path):
    # tik
    start = time.time()
    for cmd in 10 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        print("####")
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print("send: ", arg)
        print("receive: ", robot.play(**arg))
    # tok
    print("####")
    print("total time: ",time.time()-start)

if __name__ == '__main__':
    config_path = "config.json"
    
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    print("connecting")
    if not robot.connect(arg["ip"], arg["port"]):
        print("not connected")
    else:
        print("connected")
        main(robot)
    robot.close()
