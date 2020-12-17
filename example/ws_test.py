from __future__ import print_function
import sys
sys.path.append('..')
from dorna2 import dorna
import time


def main(ip, port):
    start = time.time()
    robot = dorna()
    robot.connect(ip, port)

    for cmd in 100 * ["alarm", "toollength", "input", "output", "pwm", "adc"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print(arg)
        robot.play(**arg)
        robot.wait_id(arg["id"])

    robot.close()
    print(time.time()-start)

if __name__ == '__main__':
    main("127.0.0.1", 443)
