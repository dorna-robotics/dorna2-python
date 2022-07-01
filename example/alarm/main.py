from dorna2 import Dorna
import asyncio


class call_back(object):
    """docstring for call_back"""
    def __init__(self):
        super(call_back, self).__init__()
        self.robot = Dorna()
        self.robot.register_callback(self.alarm_condition)
        

    async def alarm_condition(self, msg, sys):
        # alarm condition
        if "in0" in msg and msg["in0"] == 1:
            # de register callback
            self.robot.deregister_callback()

            # sleep for 1 seconds
            await asyncio.sleep(1)

            # activate the alarm
            self.robot.log("activating alarm")
            self.robot.alarm(1, timeout=0)
        return 0

def main(robot_cb):
    while True:
        # run the script
        result = robot_cb.robot.play_script("script.txt")
        
        # The robot is in alarm mode
        if result < 0:
            break

    # close the connection at the end    
if __name__ == '__main__':
    # ip
    ip = "localhost"

    # create the object
    cb = call_back()
    
    # connect
    cb.robot.connect(ip)
    
    # main function
    main(cb)

    # close
    cb.robot.close()