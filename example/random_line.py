import sys
sys.path.append("..") # Adds higher directory to python modules path.

from numpy import random
import dorna2

if __name__ == '__main__':
	robot = dorna2.dorna()
	robot.connect("ws://dorna:443")
	# go home
	arg = {"rel":0, "id": robot.rand_id(), "j0":0,"j1":0,"j2":0,"j3":0,"j4":0} 
	robot.jmove(**arg)
	robot.wait(arg["id"])

	# come 150 mm back toward x
	arg = {"rel":1, "id" : robot.rand_id(), "x": -150, "vel": 500, "accel": 5000, "jerk": 10000} 
	robot.lmove(**arg)
	robot.wait(arg["id"])

	# random points
	for i in range(100):
		arg = {"rel": 0, "id": i+1, "x": 350+ 100* random.rand(), "y": 100* random.rand(), "z": 206.404 + 100*random.rand()}
		print("command",i, "   arg: ", arg)
		robot.lmove(**arg)
		robot.wait(arg["id"])

	robot.ws.close()