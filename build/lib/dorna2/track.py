import threading
import queue
import time


class track_cmd(object):
    """docstring for track"""
    def __init__(self, _id):
        super(track_cmd, self).__init__()
        self.q = queue.Queue()
        self.stop = True
        self._msg = {"id": _id, "stat": None}

    def start(self):
        self.stop = False
        # socket thread
        self.track_thread = threading.Thread(target=self.track_loop)
        self.track_thread.start()

    def track_loop(self):
        while not self.stop:
            time.sleep(0)
            try:
                if not self.q.empty():
                    msg = self.q.get()
                    if msg is None:
                        self.stop = True
                    else:
                        self._msg = msg
            except:
                pass

    def msg(self):
        return dict(self._msg)

    def stat(self):
        return self._msg["stat"]

    def complete(self, time_out=0):
        if time_out > 0:
            start = time.time()

            while time.time() <= start + time_out:
                time.sleep(0)
                try:
                    stat = self.stat()
                    if any([stat > 1, stat < 0]):
                        break
                except:
                    pass
        else:
            while True:
                time.sleep(0)
                try:
                    stat = self.stat()
                    if any([stat > 1, stat < 0]):
                        break
                except:
                    pass

        return self.stat()
