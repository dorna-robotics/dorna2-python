import json
from dorna2 import dorna


def main(robot):
    robot.play_script("script.txt")

    # wait little bit before closing the connection, so the script can be executed
    robot.sleep(time=1)
    
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
