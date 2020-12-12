from random import random
import sys
sys.path.append('..')
from dorna2 import dorna


def main(ip, port):
	robot = dorna()
	robot.connect(ip, port)
	
	# go home
	print("go toward home")
	arg = {"rel":0, "id": robot.rand_id(), "j0":1,"j1":1,"j2":0,"j3":0,"j4":0} 
	robot.jmove(**arg)

	# come 150 mm back toward x
	print("go back in x direction")
	arg = {"rel":1, "id" : robot.rand_id(), "x": -200, "vel": 1000, "accel": 20000, "jerk": 40000} 
	robot.lmove(**arg)
	robot.wait(arg["id"])

	# random points
	i = 0
	while True:
		arg = {"rel": 0, "id": i+1, "x": 300+ 1* random(), "y": -100 + 1* random(), "z": 206.404 - 100 + 1*random()}
		print("command",i, "   arg: ", arg)
		robot.lmove(**arg)
		robot.wait(arg["id"])
		i += 1
	robot.ws.close()

if __name__ == '__main__':
	main("127.0.0.1", 443)