import threading
import json
from random import random
import time
from .ws import ws
from .track import track_cmd


class dorna(ws):
    """docstring for Dorna"""
    def __init__(self):
        super(dorna, self).__init__()

    def id_loop(self):
        while self.connected:
            time.sleep(0)
            msg = None
            try:
                # make sure data exists
                if self.id_q.empty():
                    continue

                # get data
                msg = self.id_q.get()

                # update according to the id
                if self.id_l and "id" in msg:
                    # find the index
                    index = self.id_l[0].index(msg["id"])

                    # update the index
                    self.id_l[1][index] = {**dict(self.id_l[1][index]), **msg}
                    # update queue
                    self.id_l[2][index].put(self.id_l[1][index])

                    # update the list
                    if "stat" in msg and msg["stat"] not in [0, 1]:
                        self.id_l[2][index].put(None)
                        for i in range(3):
                            self.id_l[i].pop(index)

            except:
                pass

        # close all the tracks
        for track in self.id_l[2]:
            track.put(None)

        # update id_l
        self.id_l = [[], [], []]

    def connect(self, host, port):
        self._connect(host, port, 1)

        # id loop
        self.id_l = [[], [], []]  # id_l[0]: index of id s, id_l[1], updated messages, id_l[2] is the queue
        self.id_thread = threading.Thread(target=self.id_loop)
        self.id_thread.start()

        # initialize
        for cmd in ["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid"]:
            arg = {"cmd": cmd, "id": self.rand_id()}
            trk = self.play(True, **arg)
            trk.complete()

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

    def play(self, track=False, message=None, **arg):
        # find msg
        if message:
            # text
            if type(message) == str:
                msg = json.loads(message)

            elif type(message) == dict:
                msg = dict(message)
        else:
            msg = dict(arg)

        # track if there is id and track is True
        trk = None
        if track and "id" in msg:
            trk = track_cmd(msg["id"])
            trk.start()

            # share the queue
            self.id_l[0].append(msg["id"])
            self.id_l[1].append({})
            self.id_l[2].append(trk.q)

        # run the command
        self.send(json.dumps(arg))
        return trk

    def jmove(self, track=False, **arg):
        arg["cmd"] = "jmove"
        return self.play(track, **arg)

    def lmove(self, track=False, **arg):
        arg["cmd"] = "lmove"
        return self.play(track, **arg)

    def rmove(self, track=False, **arg):
        arg["cmd"] = "rmove"
        return self.play(track, **arg)

    def cmove(self, track=False, **arg):
        arg["cmd"] = "cmove"
        return self.play(track, **arg)


def main():
    robot = dorna()
    robot.connect("dorna", 443)

if __name__ == '__main__':
    main()
