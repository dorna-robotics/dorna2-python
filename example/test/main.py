import time
from datetime import datetime
import json
from dorna2 import Dorna

def main(robot, f):   
    seconds = 4* 60 * 60
    i = 1
    # tik
    start = time.time()
    while time.time() < start + seconds and robot.play_script("script.txt") == 2:
        print(robot.track()["cmd"], robot.track()["merge"])
        robot.log("### round "+str(i)+" ###")
        f.write("### round "+str(i)+" ###\n")
        i += 1
        print("remain time: ", start + seconds - time.time())


    robot.log("go to the rest position")
    f.write("go to the rest position\n")
    # go to the home
    #robot.jmove(rel=0, j0=0, j1=178, j2=-140)

    robot.log("turn the motors off")
    f.write("turn the motors off\n")
    # turn of the motor
    #robot.set_motor(0)
    f.close()

if __name__ == '__main__':
    f = open("log.txt", "a")
    f.write("starting: "+str(datetime.now())+" \n")

    # dorna object
    robot = Dorna()

    # connecting
    robot.log("connecting...")
    if robot.connect("192.168.254.138"):
        robot.log("connected")
        main(robot, f)

    # close the file
    #f.write("ending: "+str(datetime.now())+ " \n")
    #f.close()
    
    # close the connection
    robot.close()
