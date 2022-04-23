from dorna2 import dorna
import json

def home(robot, joint, **kwargs)
    # move in the given direction with the given speed
    arg = {"time_out": -1, "vel": kwargs["vel_forward"], "rel":1, joint: kwargs["forward"] * kwargs["direction"]}  
    robot.jmove(**arg) # move toward the homing direction until the alarm

    # clear the alarm
    robot.alarm(alarm=0)

    for i in range(kwargs["trigger_count"])
        # move backward
        arg = {"time_out": 0, "vel": kwargs["vel_backward"], "rel":1, joint: kwargs["backward"] * kwargs["direction"]}  
        robot.jmove(**arg) # move toward the homing direction until the alarm

        # set the probe
        arg = {kwargs["probe"]: kwargs["probe_val"]}
        probe = robot.probe(**arg) # wait for the input trigger

        # halt
        robot.halt(accel=kwargs["halt_accel"])

    # set joint
    arg = {joint:  kwargs["joint_val"] + robot.get(joint)[joint]- probe[joint]}
    robot.joint(**arg)

    return True

class homing(object):
    """docstring for homing"""
    def __init__(self):
        super(homing, self).__init__()
        self.vel = [25, 2]
        self.backward = 10
        self.forward = 10000
        self.calibrate = {
            "j1":{
                "direction": -1,
                "input": "in8",
                "value": -10
            },
            "j2":{
                "direction": -1,
                "input": "in9",
                "value": -95
            },
            "j3":{
                "direction": 1,
                "input": "in11",
                "value": 98.028
            },
            "j4":{
                "direction": -1,
                "input": "in10",
                "value": -137.745
            },
            "j5":{
                "direction": 1,
                "input": "in12",
                "value": -27.21375
            },
            "j6":{
                "direction": 1,
                "input": "in12",
                "value": 67.62375
            }            
        }
    
    def _home(self, joint, vel, robot):
        s= time.time()
        arg = {"time_out": -1, "vel": vel, "rel":1, joint: self.forward * self.calibrate[joint]["direction"]}  
        robot.jmove(**arg) # move toward the homing direction
        print("time_out: -1, ", time.time()-s)    
        arg = {self.calibrate[joint]["input"]: 0}
        probe = robot.probe(**arg) # wait for the input trigger
        print("probe_result: ", probe)
        print("probe_result: ", probe[joint])
        
        # send halt
        s= time.time()
        robot.halt(accel=5)
        print("halt send")
        time.sleep(2)
        print("halt finished, ", time.time()-s)

        # set joint
        arg = {joint:  self.calibrate[joint]["value"] + robot.get(joint)[joint]- probe[joint]}
        robot.joint(**arg)

    def home(self, joint, robot):
        for vel in self.vel:
            self._home(joint, vel, robot)
            s= time.time()        
            print("move sent")
            arg = {"rel":1, "vel": self.vel[0], joint: -1* self.backward * self.calibrate[joint]["direction"]}  
            robot.jmove(**arg) # move toward the homing direction
            print("move finished")
            print("time_out: 0, ", time.time()-s)    



def main(config_path):
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    print("connecting")
    if not robot.connect(arg["ip"], arg["port"]):
        print("not connected")
        robot.close()
        return 0

    # connection confirmation    
    print("connected")

    # motors on
    robot.motor(motor=1)

    # home
    homing().home("j5", robot)
    homing().home("j6", robot)
    homing().home("j4", robot)
    homing().home("j3", robot)
    homing().home("j2", robot)
    homing().home("j1", robot)

    # close connection
    robot.close()


    print("connection closed")
if __name__ == '__main__':
    main("config.json")