import asyncio
import threading
import json
from random import random
import time
from .ws import ws
from .config import config


class dorna(ws):
    """docstring for Dorna"""
    def __init__(self):
        super(dorna, self).__init__()
        self.config = config

    def connect(self, host="localhost", port=443, time_out=5):
        # Check the connection
        print(host, port)
        if not self.server(host, port, time_out):
            return False

        # initialize
        for cmd in self.config["cmd_init"]:
            arg = {"cmd": cmd, "id": self.rand_id()}
            print(arg)
            self.play(**arg)
            self.complete(arg["id"])

        return True

    """
    wait for a given pattern ???
    ptrn = None
    ptrn = True
    ptrn = {}
    """
    def wait(self, time_out=0, **ptrn):
        self.ptrn_wait = dict(ptrn)
        if time_out > 0:
            start = time.time()
            while time.time() <= start + time_out:
                if self.ptrn_wait != ptrn:
                    return ptrn
                time.sleep(0.001)

        else:
            while True:
                if self.ptrn_wait != ptrn:
                    return ptrn 
                time.sleep(0.001)

        self.ptrn_wait = None
        return dict()


    def rand_id(self):
        return int(random() * 1000000)

    def complete(self, _id):
        return self.wait(id=_id, stat=2)

    def play(self, message=None, **arg):
        # find msg
        if message:
            # text
            if type(message) == str:
                msg = json.loads(message)

            elif type(message) == dict:
                msg = dict(message)
        else:
            msg = dict(arg)

        self.write(json.dumps(arg))

    """
    # high level commands
    """

    """
    play a text file, where each line is a command
    """
    def play_script(self, script_path=""):
        with open(script_path, 'r') as f:
            lines = f.readlines()
            num_cmd = 0
            for l in lines:
                try:
                    self.play(**json.loads(l))
                    num_cmd += 1
                except:
                    pass

        return num_cmd
    
    """
    send a motion command
    """
    def _motion(self, method, pace, complete, **arg):
        cmd = {"cmd": method, "id":self.rand_id()}
        
        if pace:
            cmd = {**dict(cmd), **self.config["pace"][pace][method]}
        cmd = {**dict(cmd), **arg}

        if complete:
            self.play(**cmd)
            return self.complete(cmd["id"])
        else:
            return self.play(**cmd) 

    """
    send jmove, rmove, lmove, cmove command
    """
    def jmove(self, pace=None, complete=True, **arg):
        return self._motion("jmove", pace, complete, **arg)

    def rmove(self, pace=None, complete=True, **arg):
        return self._motion("rmove", pace, complete, **arg)        

    def lmove(self, pace=None, complete=True, **arg):
        return self._motion("lmove", pace, complete, **arg)

    def cmove(self, pace=None, complete=True, **arg):
        return self._motion("jmove", pace, complete, **arg)

    """
    get the current value of the robot sys, based on the keys
    """
    def val(self, *arg):
        sys = dict(self.sys)
        if arg:
            return {key: sys[key] for key in arg}
        return sys

    """
    read output i, or turn it on or off
    """
    def output(self, i, out=None):
        if val:
            # set the value
            cmd = {"cmd": "output", "out"+str(i): out, "id":self.rand_id()}
            self.play(**cmd)
            self.complete(cmd["id"])
        
        return self.val("out"+str(i))

    """
    read input i
    """
    def input(self, i):
        return self.val("in"+str(i))

    """
    read pwm i, or set its parameters
    """
    def pwm(self, i, pwm=None, freq=None, duty=None):
        if any([val, freq, duty]):
            cmd = {"cmd": "pwm", "id": self.rand_id()}
            if val:
                cmd["pwm"+str(i)] = pwm
            if freq:
                cmd["freq"+str(i)] = freq
            if duty:
                cmd["duty"+str(i)] = duty
            self.play(**cmd)
            self.complete(cmd["id"])

        return self.val("pwm"+str(i), "freq"+str(i), "duty"+str(i))

    """
    read adc i
    """
    def adc(self, i):
        sys = dict(self.sys)
        return sys["adc"+str(i)]        

    """
    send a halt command
    """
    def halt(self, accel=None):
        cmd = {"cmd": "halt", "id":robot.rand_id()}
        if accel:
            cmd["accel"] = accel
        self.play(True, **cmd)
        return True 

    """
    read alarm status, set or unset alarm
    """
    def alarm(self, alarm=None):
        if alarm:
            cmd = {"cmd": "alarm", "alarm": alarm, "id":robot.rand_id()} 
            self.play(**cmd)
            self.complete(cmd["id"]) 
        
        return self.val("alarm")

    """
    sleep the controller for certain amount of time (seconds)
    """
    def sleep(self, time):
        cmd = {"cmd": "sleep", "time": time, "id":robot.rand_id()} 
        self.play(**cmd)
        self.complete(cmd["id"])

    """
    set the value of joints / and return their values in a dictionary
    """
    def joint(self, **arg):
        cmd = {"cmd": "joint", "id":robot.rand_id()}
        cmd = {**dict(cmd), **arg}
        self.play(**cmd)
        self.complete(cmd["id"])
        arg = ["j"+str(i) for i in range(8)]
        return self.val(*arg)

    """
    read motor status, set or unset alarm
    """
    def motor(self, motor=None):
        if motor:
            cmd = {"cmd": "motor", "motor": motor, "id":robot.rand_id()}
            self.play(**cmd)
            self.complete(cmd["id"])
        
        return self.val("motor")

    """
    read toollength, or set its value (mm)
    """
    def toollength(self, toollength=None):
        if toollength:
            cmd = {"cmd": "toollength", "toollength": toollength, "id":robot.rand_id()}
            self.play(**cmd)
            self.complete(cmd["id"])

        return self.val("toollength")         

    """
    read version
    """
    def version(self):
        return self.val("version") 

    """
    read uid
    """
    def uid(self):
        return self.val("uid")

    """
    on a given pattern do something
    self.on("in0" = 0, self.halt):
    ???

    
    def on(self, target, args=(), **ptrn):
        
        # define the function
        def running(self, target, args, time_out, **ptrn):
            self.wait(**ptrn)
            return eval(target+str(args))

        # run wait in thread
        thrd = threading.Thread(target=running, args=(self, target, args, **ptrn))
        thrd.start()
        return thrd
    """

def main():
    ip = "10.0.0.11"
    robot = dorna()
    print("connecting")
    robot.connect(ip)
    print("connected")
    robot.close()

if __name__ == '__main__':
    main()
