import time

from numpy import random
import dorna2

if __name__ == '__main__':
	print("press Ctrl-C to kill the program")
	robot = dorna2.dorna()
	robot.connect("ws://dorna:443")

	try:
		# go home
		arg = {"cmd": "rmove", "rel":0, "id": robot.rand_id(), "j0":0,"j1":0,"j2":0,"j3":0,"j4":0, "vel": 0.8, "accel": 0.3 ,"jerk": 0.07} 
		print("going home ->")
		robot.play(**arg)
		robot.wait(arg["id"])		
		
		# random points
		i = 1
		while True:
			j0 = random.uniform(-90, 90)
			j1 = random.uniform(-45, 90)
			j2 = random.uniform(max(-90, -2.7*j1 - 60), 90)
			j3 = random.uniform(-90, 90)
			j4 = 0

			arg = {"cmd": "rmove","rel":0, "id": robot.rand_id(), "j0": j0, "j1": j1, "j2": j2, "j3": j3, "j4": j4} 
			print("command",i, "   arg: ", arg)
			robot.play(**arg)
			robot.wait(arg["id"])	
			i += 1
			time.sleep(0.5)
	except KeyboardInterrupt:
		pass
	robot.ws.close()