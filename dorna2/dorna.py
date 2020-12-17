from __future__ import print_function
import json
from random import random
import time
from .ws import ws


class dorna(ws):
    """docstring for Dorna"""
    def __init__(self):
        super(dorna, self).__init__()

    def connect(self, host, port, wait=1, init=True):
        self._connect(host, port, wait)
        # initialize
        if init:
            for cmd in ["alarm", "toollength", "input", "output", "pwm", "adc"]:
                arg = {"cmd": cmd, "id": self.rand_id()}
                self.play(**arg)
                self.wait(id=arg["id"], stat=2)

    def rand_id(self):
        return int(random() * 1000000)

    def wait(self, time_out=0, **arg):
        if time_out > 0:
            start = time.time()

            while time.time() <= start + time_out:
                try:
                    sys = dict(self.sys)
                    if all([sys[x] == arg[x] for x in arg]):
                        return True
                except:
                    pass
        else:
            while True:
                try:
                    sys = dict(self.sys)
                    if all([sys[x] == arg[x] for x in arg]):
                        return True
                except:
                    pass
        return False

    def wait_id(self, _id):
        self.wait(id=_id, stat=2)

    def play_file(self, file_path=""):
        with open(file_path, 'r') as f:
            lines = f.readlines()
            for l in lines:
                self.play(**json.loads(l))

    def play(self, txt="", **arg):
        try:
            arg = {**json.loads(txt), **arg}
        except:
            pass
        self.send(json.dumps(arg))

    def jmove(self, **arg):
        arg["cmd"] = "jmove"
        self.play(**arg)

    def lmove(self, **arg):
        arg["cmd"] = "lmove"
        self.play(**arg)

    def rmove(self, **arg):
        arg["cmd"] = "rmove"
        self.play(**arg)

    def cmove(self, **arg):
        arg["cmd"] = "cmove"
        self.play(**arg)


def main():
    robot = dorna()
    robot.connect("dorna", 443)

if __name__ == '__main__':
    main()
