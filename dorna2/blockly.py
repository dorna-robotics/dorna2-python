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
