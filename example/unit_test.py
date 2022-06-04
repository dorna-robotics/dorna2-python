import time
import json
from dorna2 import Dorna

def main(config_path):
    seconds = 4 * 60 * 60
    i = 1
    # tik
    start = time.time()
    while time.time() < start + seconds:
        robot.play_script("unit_test_script.txt")
        robot.log("### round "+str(i)+" ###")
        i += 1

if __name__ == '__main__':
    config_path = "config.json"
    
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
