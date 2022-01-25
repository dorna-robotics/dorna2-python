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
        print(host, port)
        if not self.server(host, port, time_out):
            return False

        # initialize
        for cmd in self.config["cmd_init"]:
            self.cmd(cmd)

        return True

    """
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

    """
    complete: wait for the completion or not
    time_out: the maximum amount of time (seconds), the API waits for the command completion, 
                -1: does not wait
                0: waits forever
                positive: waits for few maximum given seconds
    msg: command in form of text or dict
    """
    def play(self, time_out=0, msg=None, **kwargs):
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
        if time_out >=0:
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
    play a text file, where each line is a command
    """
    def play_script(self, script_path=""):
        with open(script_path, 'r') as f:
            lines = f.readlines()
            num_cmd = 0
            for l in lines:
                try:
                    self.play(time_out=-1, message=l)
                    num_cmd += 1
                except:
                    pass

        return num_cmd
    
    """
    send a motion command
    """
    def _motion(self, method, pace, time_out, **kwargs):
        cmd = {"cmd": method, "id":self.rand_id()}
        
        if pace:
            cmd = {**dict(cmd), **self.config["pace"][pace][method]}
        cmd = {**dict(cmd), **kwargs}

        return self.play(time_out=time_out, **cmd)

    """
    send jmove, rmove, lmove, cmove command
    """
    def jmove(self, pace=None, time_out=0, **kwargs):
        return self._motion("jmove", pace, time_out, **kwargs)

    def rmove(self, pace=None, time_out=0, **kwargs):
        return self._motion("rmove", pace, time_out, **kwargs)        

    def lmove(self, pace=None, time_out=0, **kwargs):
        return self._motion("lmove", pace, time_out, **kwargs)

    def cmove(self, pace=None, time_out=0, **kwargs):
        return self._motion("jmove", pace, time_out, **kwargs)

    """
    return one value
    """
    def val(self, key):
        return self.get(key)[key]

    """
    return a dictionary based on keys
    """        
    def get(self, *args):
        sys = dict(self.sys)
        if args:
            return {key: sys[key] for key in args}
        return sys

    # set a parameter and wait for its reply from the controller
    def cmd(self, cmd, time_out=0, **arg):
        cmd = {**{"cmd": cmd}, **arg}
        return self.play(time_out=time_out, **cmd)

    """
    read output i, or turn it on or off
    """
    def output(self, **arg):
        return self.cmd("output", **arg)

    """
    read pwm i, or set its parameters
    """
    def pwm(self, **arg):
        return self.cmd("pwm", **arg)

    """
    read input i
    """
    def input(self, *arg):
        return self.cmd("input")

    """
    read adc i
    """
    def adc(self, *arg):
        return self.cmd("adc")

    def probe(self, **arg):
        return self.cmd("probe", **arg)
    
    """
    send a halt command
    """
    def halt(self, **arg):
        return self.cmd("halt", **arg)

    """
    read alarm status, set or unset alarm
    """
    def alarm(self, **arg):
        return self.cmd("alarm", **arg)

    """
    sleep the controller for certain amount of time (seconds)
    """
    def sleep(self, **arg):
        return self.cmd("sleep", **arg)

    """
    set the value of joints / and return their values in a dictionary
    """
    def joint(self, **arg):
        return self.cmd("joint", **arg)

    """
    read motor status, set or unset alarm
    """
    def motor(self, **arg):
        return self.cmd("motor", **arg)

    """
    read toollength, or set its value (mm)
    """
    def toollength(self, **arg):
        return self.cmd("toollength", **arg)        

    """
    read version
    """
    def version(self, **arg):
        return self.cmd("version", **arg)

    """
    read uid
    """
    def uid(self, **arg):
        return self.cmd("uid", **arg)
