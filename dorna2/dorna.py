import json
from random import random
import time
from dorna2.ws import ws
from dorna2.config import config

class dorna(ws):
    """docstring for Dorna"""
    def __init__(self, config=config):
        super(dorna, self).__init__()
        self.config = config

    def connect(self, host="localhost", port=443, time_out=5):
        # Check the connection
        print(host, ":", port)
        if not self.server(host, port, time_out):
            return False

        # initialize
        for cmd in self.config["cmd_init"]:
            self.cmd(cmd)

        return True

    def log(self, msg=""):
        return print(msg, flush=True)        
    
    def rand_id(self):
        return int(random() * 1000000)

    """
    complete: wait for the completion or not
    time_out: the maximum amount of time (seconds), the API waits for the command completion, 
                negative (e.g. -1): wait for the command completion or error
                0: no waiting
                positive: wait for maximum time_out seconds to complete the command
    msg: command in form of text or dict
    **kwargs: Other parameters associated to the play
    --------
    return a dictionary formed by the union of the messages associated to this command 
    """
    def play(self, time_out=-1, msg=None, **kwargs):
        rtn = dict()
        # find msg
        if msg:
            # text
            if type(msg) == str:
                msg = json.loads(msg)

            elif type(msg) == dict:
                msg = dict(msg)
        else:
            msg = dict(kwargs)

        # set the track
        if time_out >0 or time_out < 0:
            # check the id
            try:
                type(msg["id"]) == int
            except Exception as ex:
                msg["id"] = self.rand_id()

            # send and track messages
            try:
                # set the tracking id
                self.track = {"id": msg["id"], "msgs": []}
                
                # write the message
                self.write(json.dumps(msg))

                # track
                if time_out > 0:
                    start = time.time()
                    while time.time() <= start + time_out and self.track["id"] == msg["id"]:
                        time.sleep(0.001)
                else:
                    while self.track["id"] == msg["id"]:
                        time.sleep(0.001)
                
                # form the union update
                for i in range(len(self.track["msgs"])):
                    rtn = {**rtn, **self.track["msgs"][i]}
            except Exception as ex:
                pass
        else:
            # write the message
            self.write(json.dumps(msg))
        return rtn


    """
    # high level commands
    """

    """
    play a text file, where each line is a valid command, if a command is not valid, it will skip it and send the next one
    script_path: path to the script file
    time_out= if you want to wait for the completion of this block of scripts or not
        negative (-1): waits for its completion
        0: no wait
        positive: wait for maximum of time_out seconds
    --------
    return number valid commands that was sent  
    """
    def play_script(self, script_path="", time_out=0):
        with open(script_path, 'r') as f:
            lines = f.readlines()
            num_cmd = 0
            for l in lines:
                try:
                    self.play(time_out=0, msg=l)
                    num_cmd += 1
                except:
                    pass
        self.sleep(time_out=time_out, sleep=0)
        return num_cmd
    
    """
    send a motion command
    """
    def _motion(self, method, pace, **kwargs):
        cmd = {"cmd": method}
        
        if pace:
            cmd = {**dict(cmd), **self.config["pace"][pace][method]}
        cmd = {**dict(cmd), **kwargs}

        return self.play(**cmd)

    """
    send jmove, rmove, lmove, cmove command
    """
    def jmove(self, pace=None, **kwargs):
        return self._motion("jmove", pace, **kwargs)

    def rmove(self, pace=None, **kwargs):
        return self._motion("rmove", pace, **kwargs)        

    def lmove(self, pace=None, **kwargs):
        return self._motion("lmove", pace, **kwargs)

    def cmove(self, pace=None, **kwargs):
        return self._motion("cmove", pace, **kwargs)

    """
    return a dictionary based on keys
    if no *args is present it will return a copy of sys
    """        
    def get(self, *args):
        sys = dict(self.sys)
        if args:
            return {key: sys[key] for key in args}
        return sys

    """
    return one value based on the key
    """
    def val(self, key):
        return self.get(key)[key]

    # It is a shorten version of play, 
    # set a parameter and wait for its reply from the controller
    def cmd(self, cmd, **kwargs):
        cmd = {**{"cmd": cmd}, **kwargs}
        return self.play(**cmd)

    """
    read output i, or turn it on or off
    """
    def output(self, **kwargs):
        return self.cmd("output", **kwargs)

    """
    read pwm, or set its parameters
    """
    def pwm(self, **kwargs):
        return self.cmd("pwm", **kwargs)

    """
    read input
    """
    def input(self, **kwargs):
        return self.cmd("input", **kwargs)

    """
    read adc
    """
    def adc(self, **kwargs):
        return self.cmd("adc", **kwargs)

    def probe(self, **kwargs):
        return self.cmd("probe", **kwargs)
    
    """
    send a halt command
    """
    def halt(self, time_out = 0, **kwargs):
        return self.cmd("halt", time_out = time_out, **kwargs)

    """
    read alarm status, set or unset alarm
    """
    def alarm(self, **kwargs):
        return self.cmd("alarm", **kwargs)

    """
    sleep the controller for certain amount of time (seconds)
    """
    def sleep(self, **kwargs):
        return self.cmd("sleep", **kwargs)

    """
    set the value of joints / and return their values in a dictionary
    """
    def joint(self, **kwargs):
        return self.cmd("joint", **kwargs)

    def pose(self):
        return self.get("x", "y", "z", "a", "b", "c", "d", "e")
    """
    read motors status, activate or deactivate the motors
    """
    def motor(self, **kwargs):
        return self.cmd("motor", **kwargs)    

    """
    read toollength, or set its value (mm)
    """
    def toollength(self, **kwargs):
        return self.cmd("toollength", **kwargs)        

    """
    read version
    """
    def version(self, **kwargs):
        return self.cmd("version", **kwargs)

    """
    read uid
    """
    def uid(self, **kwargs):
        return self.cmd("uid", **kwargs)
