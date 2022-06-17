import time
from dorna2 import Dorna
import random
import time

id_list = None
flag = None

async def track_id(self, msg, sys):
    global id_list
    global flag
    if all(["stat" in msg, "id" in msg]):
        _id = int(msg["id"] - 2)
        _stat = int(msg["stat"])

        # id range
        if _id > 99:
            self.log("out of range: "+ str(_id + 2))
            return False

        id_list[_stat][_id] +=1

        # multiple stat
        if id_list[_stat][_id] > 1:
            flag = 1
            self.log("stat: " + str(_stat) + " id: " + str( _id + 2))
            return False

        # check for 0, 1, 2
        if id_list[_stat][_id] !=  id_list[max(0, _stat-1)][_id]:
            flag=2
            self.log("stat: " + str(_stat) + " id: " + str( _id + 2))
            return False

        # check for stat pattern
        if id_list[_stat][_id] != id_list[_stat][max(0, _id-1)]:
            flag=3
            self.log("stat: " + str(_stat) + " id: " + str( _id + 2))
            return False

        if id_list[2][-1] == 1:
            flag=4
            return False 


def init_id_list(num_cmd):
    return [[0 for i in range(num_cmd)] for i in range(3)]


def random_pose(x, y, z, a, b):
    return [x + 0.1 * random.random(), y + 0.1 * random.random(), z + 0.1 * random.random(), a + 0.1 * random.random(), b + 0.1 * random.random()]

def main(robot):
    num_loop = 5000
    num_cmd = 100
    global id_list
    global flag

    # get the initial point
    x0, y0, z0, a0, b0 = 0, 0, 0, 0, 0
    
    time.sleep(2)
    for i in range(num_loop):
        robot.log("round "+str(i))
        # init id_list
        id_list = init_id_list(num_cmd)
        flag = None

        # set_callback
        robot.register_callback(track_id)
        
        # create 100 messages
        for j in range(2, num_cmd+2):
            robot.lmove(rel=0, vel=200, accel=400, jerk=8000, x=x0 +0.05*((-1)**j), y=y0 +0.05*((-1)**j), z=z0 +0.05*((-1)**j), a=a0 +0.05*((-1)**j), b=b0 +0.05*((-1)**j), id=j, timeout=0)

        while not flag:  
            time.sleep(0.001)        

        if flag != 4:
            robot.log("error flag: " + str(flag))
            robot.log(id_list)
            return flag


if __name__ == '__main__':
    # dorna object
    robot = Dorna()
    robot.log("starting")

    # connecting
    robot.log("connecting...")
    if robot.connect():
        robot.log("connected")
        main(robot)

    robot.log("end")
    # close the connection
    robot.close()
