from __future__ import print_function
import sys
import json
sys.path.append('..')
from dorna2 import dorna


def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    robot.connect(arg["ip"], arg["port"])

    robot.play_script("script.txt")
    robot.wait(id=1000, stat=2)  # end of script

    robot.close()

if __name__ == '__main__':
    main("config.json")
