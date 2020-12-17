from __future__ import print_function
import sys
sys.path.append('..')
from dorna2 import dorna


def wait_input(ip, port):
    robot = dorna()
    robot.connect(ip, port)
    robot.wait(in0=1)
    robot.close()

if __name__ == '__main__':
    wait_input("127.0.0.1", 443)
