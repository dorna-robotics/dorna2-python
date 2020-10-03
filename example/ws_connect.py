"""
import sys 
sys.path.append('..')
"""
from dorna2 import dorna


def main():
	robot = dorna()
	# check for successful connection
	if not robot.connect("ws://dorna:443"):
		return False

	for cmd in ["alarm", "toollength", "input", "output", "pwm", "adc"]:
		arg = {"cmd": cmd, "id": robot.rand_id()}
		print(arg)
		robot.play(**arg)
		robot.wait(arg["id"])		

	robot.ws.close()	

if __name__ == '__main__':
	main()
