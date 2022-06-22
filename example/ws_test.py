import time
import json
from dorna2 import Dorna

def main(robot):
    # tik
    start = time.time()
    for cmd in 10 * ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
        robot.log(robot.cmd(cmd))
    # tok
    robot.log(time.time()-start)

if __name__ == '__main__':
    config_path = "config.json"
    
    for i in range(1):
        # arguments
        with open(config_path) as json_file:
            arg = json.load(json_file)

        robot = Dorna()
        robot.log("connecting")
        if not robot.connect(arg["ip"], arg["port"]):
            robot.log("not connected")
        else:
            robot.log("connected")
            main(robot)
        robot.close()
