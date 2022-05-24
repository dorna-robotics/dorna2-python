import time
import json
from dorna2 import Dorna

def main(config_path):
    # tik
    start = time.time()
    for cmd in 10 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        print("####")
        print("receive: ", robot.cmd(cmd))
    # tok
    print("####")
    print("total time: ",time.time()-start)

if __name__ == '__main__':
    config_path = "config.json"
    
    for i in range(1):
        # arguments
        with open(config_path) as json_file:
            arg = json.load(json_file)

        robot = Dorna()
        print("connecting")
        if not robot.connect(arg["ip"], arg["port"]):
            print("not connected")
        else:
            print("connected")
            main(robot)
        robot.close()
