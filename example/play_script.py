import json
from numpy import random
import dorna2

def main():
	robot = dorna2.dorna()
	if not robot.connect("ws://dorna:443"):
		return False

	# go home
	arg = {"cmd": "rmove", "rel":0, "id": robot.rand_id(), "j0":0,"j1":0,"j2":0,"j3":0,"j4":0, "vel": 0.7, "accel": 0.3 ,"jerk": 0.1} 
	print("going home ->")
	robot.play(**arg)
	robot.wait(arg["id"])		


	with open("script.txt", "r") as f:
		for line in f:
			arg = json.loads(line[0:-1])
			robot.play(**arg)
			print(arg)
			arg = {"cmd":"sleep","time":0.3,"id":robot.rand_id()}
			robot.play(**arg)

	#robot.wait(arg["id"])

	robot.ws.close()

if __name__ == '__main__':
	main()