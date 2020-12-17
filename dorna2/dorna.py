from __future__ import print_function
import json
import time
from .ws import ws

class dorna(ws):
    """docstring for Dorna"""
    def __init__(self):
        super(dorna, self).__init__()

    def rand_id(self):
        return int(time.time() * 10000) % 1000000

    def wait(self, _id, prnt=False, stop=False):
        while not stop:
            try:
                sys = dict(self.sys)
                if "stat" in sys and "id" in sys \
                        and sys["stat"] == 2 and sys["id"] == _id:
                    break

            except Exception as ex:
                print("(wait) Error: ", ex)
                pass

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
    (robot.connect("dorna", 443)

if __name__ == '__main__':
    main()