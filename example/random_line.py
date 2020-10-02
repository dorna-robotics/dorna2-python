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
	arg = {"rel":1, "id" : robot.rand_id(), "x": -200, "vel": 1000, "accel": 20000, "jerk": 40000} 
	robot.lmove(**arg)
	robot.wait(arg["id"])

	# random points
	for i in range(10000):
		arg = {"rel": 0, "id": i+1, "x": 300+ 100* random.rand(), "y": -100 + 200* random.rand(), "z": 206.404 - 100 + 200*random.rand()}
		print("command",i, "   arg: ", arg)
		robot.lmove(**arg)
		robot.wait(arg["id"])

	robot.ws.close()