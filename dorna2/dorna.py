from __future__ import print_function
import json
from random import random
import time
from .ws import ws


class dorna(ws):
    """docstring for Dorna"""
    def __init__(self):
        super(dorna, self).__init__()

    def connect(self, host, port):
        self._connect(host, port, 1)
        # initialize
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

    def play_script(self, script_path=""):
        with open(script_path, 'r') as f:
            lines = f.readlines()
            i = 0
            for l in lines:
                try:
                    self.play(**json.loads(l))
                    i += 1
                except:
                    pass
            return i

    def play(self, message=None, **arg):
        if message:
            # text
            if type(message) == str:
                return self.send(message)
            elif type(message) == dict:
                return self.send(json.dumps(message))
        else:
            return self.send(json.dumps(arg))

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
