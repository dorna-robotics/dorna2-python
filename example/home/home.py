from dorna2 import dorna
import json
from config import config

def home(robot, joint, **kwargs):
    print(kwargs)
    # move in the given direction with the given speed
    arg = {"time_out": -1, "vel": kwargs["vel_forward"], "rel":1, joint: kwargs["forward"] * kwargs["direction"]}  
    robot.jmove(**arg) # move toward the homing direction until the alarm

    # alarm
    print(alarm)
    
    # clear the alarm
    robot.alarm(alarm=0)

    for i in range(kwargs["trigger_count"]):
        # move backward
        arg = {"time_out": 0, "vel": kwargs["vel_backward"], "rel":1, joint: kwargs["backward"] * kwargs["direction"]}  
        robot.jmove(**arg) # move toward the homing direction until the alarm

        # set the probe
        arg = {kwargs["iprobe"]: kwargs["iprobe_val"]}
        iprobe = robot.iprobe(**arg) # wait for the input trigger

        # halt
        robot.halt(accel=kwargs["halt_accel"])

    # set joint
    arg = {joint:  kwargs["joint_val"] + robot.get(joint)[joint]- iprobe[joint]}
    robot.joint(**arg)

    return True

if __name__ == '__main__':        
    robot = dorna()
    robot.connect("192.168.254.126", 443)

    print("connected")
    home(robot,"j5", **config["j5"])
    