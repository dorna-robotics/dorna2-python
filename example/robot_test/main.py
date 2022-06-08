import time
import json
from dorna2 import Dorna

def main(robot):
    # turn the motors on
    robot.set_motor(1)

    # go to the stat position
    robot.jmove(rel=0, j0=0, j1=0, j2=0, j3=0, j4=0)

    seconds = 1 * 60 * 60
    i = 1
    # tik
    start = time.time()
    while time.time() < start + seconds and robot.play_script("script.txt") == 2:
        robot.log("### round "+str(i)+" ###")
        i += 1

    robot.log("go to the rest position")
    # go to the home
    robot.jmove(rel=0, j0=0, j1=178, j2=-140)

    robot.log("turn the motors off")
    # turn of the motor
    robot.set_motor(0)

if __name__ == '__main__':
    # dorna object
    robot = Dorna()
    robot.log("starting")

    # connecting
    robot.log("connecting...")
    if robot.connect("localhost"):
        robot.log("connected")
        main(robot)

    robot.log("end")
    # close the connection
    robot.close()
