import json
from dorna2 import dorna
"""
Use play method to send command to the robot.
There are many ways to call this method
"""
def main(robot):
	# send a command and wait for its completion, by setting the time_out parameter to 0. The default value for time_out is 0.
	arg = {"cmd": "jmove", "rel": 1, "j0":50, "vel":10}
	robot.play(time_out=0, **arg) # this is identical to robot.play(**arg)
	print("Motion is completed")

	# send a command and do not wait for its completion by setting the time_out parameter to a negative number
	robot.play(time_out=-1, **arg)
	print("Motion is still running")

	# send a command and wait maximum of 2 seconds for its completion
	robot.play(time_out=2, **arg)
	print("We waited maximum of 2 seconds for the motion completion") 	

	# send a command in python dictionary format
	robot.play(msg=arg)
	print("Motion is completed")

	# send a command in text format
	robot.play(msg='{"cmd": "jmove", "rel": 1, "j0":50, "vel":10}') 	
	print("Motion is completed")

if __name__ == '__main__':
    config_path = "config.json"
    
    # arguments
    with open(config_path) as json_file:
        arg = json.load(json_file)

    robot = dorna()
    print("connecting")
    if not robot.connect(arg["ip"], arg["port"]):
        print("not connected")
    else:
    	print("connected")
    	main(robot)
    robot.close()
