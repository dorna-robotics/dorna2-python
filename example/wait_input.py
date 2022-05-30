import json
from dorna2 import dorna

def main(robot):
    # use probe to wait for input0 turns 1
    robot.probe(in0=1)

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
