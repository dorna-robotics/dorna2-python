from __future__ import print_function
import sys
sys.path.append('..')
from dorna2 import dorna


def play_file(ip, port):
    robot = dorna()
    robot.connect(ip, port)
    robot.play_file("script.txt")
    robot.wait(id=1000, stat=2)
    robot.close()

if __name__ == '__main__':
    play_file("127.0.0.1", 443)
