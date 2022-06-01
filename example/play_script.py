import json
from dorna2 import Dorna


def main(robot):
    robot.play_script("script.txt")
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
