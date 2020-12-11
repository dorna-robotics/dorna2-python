from numpy import random
import dorna2

def main():
	robot = dorna2.dorna()
	if not robot.connect("ws://192.168.1.8:443"):
		return False

	# go home
	arg = {"cmd": "rmove", "rel":0, "id": robot.rand_id(), "j0":0,"j1":0,"j2":0,"j3":0,"j4":0, "vel": 0.5, "accel": 0.3 ,"jerk": 0.1} 
	print("going home ->")
	robot.play(**arg)
	robot.wait(arg["id"])		
	
	# random points
	for i in range(10000):
		j0 = random.uniform(-90, 90)
		j1 = random.uniform(-45, 90)
		j2 = random.uniform(max(-90, -2.7*j1 - 60), 90)
		j3 = random.uniform(-90, 90)
		j4 = 0

		arg = {"cmd": "rmove","rel":0, "id": i+1, "j0": j0, "j1": j1, "j2": j2, "j3": j3, "j4": j4} 
		print("command",i, "   arg: ", arg)
		robot.play(**arg)
		robot.wait(arg["id"])	

	robot.ws.close()

if __name__ == '__main__':
	main()