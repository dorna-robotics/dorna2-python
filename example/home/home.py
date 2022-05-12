from dorna2 import Dorna
import json
from config import config
import time

def home(robot, index, **kwargs):
    # lower the threshold
    arg = {"cmd":"pid","threshold":10,"duration":5}
    robot.play(**arg)

    joint = "j"+str(index)
    # move in the given direction with the given speed
    arg = {"vel": kwargs["vel_forward"], "rel":1, joint: kwargs["forward"] * kwargs["direction"]}  
    alarm_move = robot.jmove(**arg) # move toward the homing direction until the alarm
    
    print(alarm_move)
    # sleep for a short amount
    time.sleep(0.2)

    # clear the alarm
    robot.alarm(0)

    # bring the threshold to normal
    arg = {"cmd":"pid","threshold":75,"duration":3000}
    robot.play(**arg)
    
    for i in range(kwargs["trigger_count"]):
        # move backward
        arg = {"time_out": 0, "vel": kwargs["vel_backward"], "rel":1, joint: kwargs["backward"] * kwargs["direction"]}  
        robot.jmove(**arg) # move toward the homing direction until the alarm

        # set the probe
        print("set the iprobe")
        arg = {kwargs["iprobe"]: kwargs["iprobe_val"]}
        iprobe = robot.iprobe(index, kwargs["iprobe_val"]) # wait for the input trigger

        print("iprobe: ", iprobe)
        
        # halt
        robot.halt(kwargs["halt_accel"])

    time.sleep(1)

    # set joint
    joint_assignment = kwargs["joint_val"] + robot.val(joint) - iprobe[index]
    #robot.joint(index, joint_assignment)

    # go to a fixed position
    robot.jmove(rel=0, j5=-5)

    return True
    
if __name__ == '__main__':
    index = 5        
    robot = Dorna()
    robot.connect("192.168.254.126")

    print("connected")
    for i in range(100):
        home(robot, index, **config["j"+ str(index)])
        time.sleep(2)
    robot.close()
    