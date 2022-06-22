"""
sudo python3 home.py -i 5 -p 10.0.0.4
"""
from dorna2 import Dorna
import json
from config import config
import time
import argparse

"""
id starts from 2,000,000
homing process for joint i starts
2,000,0i0: set pid
2,000,0i1: alarm motion
2,000,0i2: clear alarm
2,000,0i3: reset pid
2,000,0i4: move backwards and set iprobe
2,000,0i5: halt
2,000,0i6: joint assignment
2,000,0i7: error, clear alarm
2,000,0i8: error, reset pid 
"""
def home(robot, index, val, **kwargs):
    # activate the motors
    result = robot.set_motor(1)
    if result != 2:
        # error happened
        return home_error_handling(robot, index)     
    # set thereshould
    id = 2000000+10*index # 0
    result = robot.set_pid(threshold=10, duration=5, id=id)
    if result != 2:
        # error happened
        return home_error_handling(robot, index) 
    # set joint
    id += 1 # 1
    joint = "j"+str(index)
    # move in the given direction with the given speed
    arg = {"vel": kwargs["vel_forward"], "rel":1, joint: kwargs["forward"] * kwargs["direction"], "id": id}  
    result = robot.jmove(**arg) # move toward the homing direction until the alarm
    if result >= 0:
        # error happened
        return home_error_handling(robot, index) 

    # sleep
    time.sleep(0.2)

    # clear the alarm
    id += 1 # 2
    result = robot.set_alarm(0, id=id)
    if result != 2:
        # error happened
        return home_error_handling(robot, index)

    # reset pid
    id += 1 # 3
    result = robot.reset_pid(id=id)
    if result != 2:
        # error happened
        return home_error_handling(robot, index) 
    
    for i in range(kwargs["trigger_count"]):
        # move backward
        arg = {"timeout": 0, "vel": kwargs["vel_backward"], "rel":1, joint: kwargs["backward"] * kwargs["direction"]}  
        robot.jmove(**arg) # move toward the homing direction until the alarm

        # set the probe
        id += 1 # 4
        iprobe = robot.iprobe(index, kwargs["iprobe_val"], id=id) # wait for the input trigger

        # halt
        id += 1 # 5
        result = robot.halt(kwargs["halt_accel"], id=id)
        if result != 2:
            # error happened
            return home_error_handling(robot, index)

    time.sleep(1)

    # set joint
    joint_assignment = val + robot.val(joint) - iprobe[index]
    id += 1 # 6
    result = robot.set_joint(index, joint_assignment, id=id)
    if result != 2:
        # error happened
        return home_error_handling(robot, index)

    #arg = {"rel": 0, joint: kwargs["stop"]}
    #robot.jmove(**arg)
    return True


def home_error_handling(robot, index):
    id = 2000000+10*index+7 # 7
    robot.set_alarm(0, id=id)
    id += 1 # 8
    robot.reset_pid(id=id)
if __name__ == '__main__':
    # Initialize parser
    parser = argparse.ArgumentParser()

    # Adding optional argument
    parser.add_argument("--Index")
    parser.add_argument("--Host")
    parser.add_argument("--Value")
    # Read arguments from command line
    args = parser.parse_args()
    
    # assign index
    index = int(args.Index)        
    host = args.Host
    val = float(args.Value)

    robot = Dorna()
    robot.connect(host)

    robot.log("connected")
    for i in range(1):
        home(robot, index, val, **config["j"+ str(index)])
        time.sleep(1)
    robot.close()
    