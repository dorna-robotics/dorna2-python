from __future__ import print_function
import sys
sys.path.append('..')
from dorna2 import dorna
import time


def main(ip, port):
    start = time.time()
    robot = dorna()
    robot.connect(ip, port)
    for cmd in 10 * ["alarm", "toollength", "input", "output", "pwm", "adc"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print(arg)
        trk = robot.play(True, **arg)
        trk.complete()

    robot.close()
    print(time.time()-start)

if __name__ == '__main__':
    main("10.0.0.3", 443)
