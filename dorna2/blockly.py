cmd_list = ["jmove", "lmove", "cmove", "rmove", "halt", "alarm", "sleep", "input", "probe", "output", "pwm", "adc", "joint", "motor", "toollength", "version", "uid"]
output_list = ["out0", "out1", "out2", "out3", "out4", "out5", "out6", "out7", "out8", "out9", "out10", "out11", "out12", "out13", "out14", "out15"]
pwm_list = ["pwm0", "pwm1", "pwm2", "pwm3", "pwm4", "duty0", "duty1", "duty2", "duty3", "duty4", "freq0", "freq1", "freq2", "freq3", "freq4"]
probe_list = ["in0", "in1", "in2", "in3", "in4", "in5", "in6", "in7", "in8", "in9", "in10", "in11", "in12", "in13", "in14", "in15"]

# get_list is also similar to val_list
val_list =   ["j0", "j1", "j2", "j3", "j4", "j5", "j6", "j7", "x", "y", "z", "a", "b", "c", "d", "e"]
            + output_list + pwm_list + probe_list + ["adc0", "adc1", "adc2", "adc3", "adc4"]
            + ["id", "stat", ]
            + ["time", "motor", "version", "uid"]
            + ["queue", ]
            + [ "alarm", "err0", "err1", "err2", "err3", "err4", "err5", "err6", "err7"]
            + ["cmd" , "vel", "accel", "jerk", "rel"]
            + ["turn", "dim", "space", "mj0", "mj1", "mj2", "mj3", "mj4", "mj5", "mj6", "mj7", "mx", "my", "mz", "ma", "mb", "mc", "md", "me"]

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

    """
    host: "localhost", string
    port: 443, int
    ---
    True or False
    """
    def connect(self, host="localhost", port=443, time_out=5):

    """
    msg: any type
    """
    def log(self, msg=""):
    
    """
    ---
    int
    """
    def rand_id(self):

    """
    time_out = -1, float number
    cmd = list of valid cmd
    ----
    dictionary
    """
    def play(self, time_out=-1, msg=None, **kwargs):


    """
    script_path: string
    time_out: 0
    """
    def play_script(self, script_path="", time_out=0):
    
    """
    pace = 20%, percentage
    rel = 0, 0 or 1
    j0 = 0, float
    """
    def jmove(self, **kwargs):

    """
    pace = 20%, percentage
    rel = 0, 0 or 1
    x = 0, float    
    """
    def lmove(self, **kwargs):

    """
    pace = 20%, percentage
    rel = 0, 0 or 1
    turn = 0, int
    x = 20, float
    mx= 10, float    
    """
    def cmove(self, **kwargs):

    """
    in0
    """
    def val(self, key):

    """
    out0: 0, 0 or 1
    ...
    ---
    dictionary
    """
    def output(self, **kwargs):
        return self.cmd("output", **kwargs)

    """
    pwm0: 0, 0 or 1
    duty0: double, between 0 and 100
    freq0: double, between 120,000,000
    ...
    ---
    dictionary
    """
    def pwm(self, **kwargs):

    """
    no parameter
    ---
    dictionary
    """
    def input(self, *args):

    """
    no parameter
    ---
    dictionary
    """
    def adc(self, **kwargs):

    """
    in0 = 0, 0 or 1
    ...
    ----
    dictionary
    """
    def probe(self, **kwargs):
    
    """
    ---
    return dictionary
    """
    def halt(self, **kwargs):

    """
    alarm = 0, 0 or 1
    ----
    dictionary
    """
    def alarm(self, **kwargs):

    """
    time: 1, float positive
    ---
    return dictionary 
    """
    def sleep(self, **kwargs):

    """
    j0 = 0 , float
    ...
    ---
    dictionary
    """
    def joint(self, **kwargs):

    """
    ---
    return dictionary
    """
    def pose(self):
    """
    motor: 1, 0 or 1
    ---
    dictionary
    """
    def motor(self, **kwargs):

    """
    toollength: 0, double > 0
    ---
    return dictionary
    """
    def toollength(self, **kwargs):

    """
    ---
    retun dictionary
    """
    def version(self, **kwargs):

    """
    ---
    return uid
    """
    def uid(self, **kwargs):
