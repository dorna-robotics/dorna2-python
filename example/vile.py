from dorna2 import Dorna
import numpy as np

class lattice(object):
	"""docstring for lattice"""
	def __init__(self, O, A, B, num_a, num_b):
		super(lattice, self).__init__()
		self.O = np.array(O)
		self.A = np.array(A) 
		self.B = np.array(B)
		self.num_a = num_a
		self.num_b = num_b


	def point(self, i, j):
		return (i/(num_a-1)) * (self.A - self.O)+ (j/(num_b-1)) * (self.B - self.O) + self.O 

def pick_drop(robot, point):
	z_up = 120
	z_down = -25
	#z_down = -40

	# open gripper
	robot.output(0, 0)

	# go to the target point
	robot.lmove(rel=0, x=point[0], y=point[1], z=z_up, a=-90, c=point[5],  vel=200)

	# get the joins
	joint = robot.joint()

	# adjust b
	b = 45 - joint[0]
	robot.lmove(rel=0, b=b)

	# go down
	robot.lmove(rel=0, z=z_down)

	# close the gripper
	robot.output(0, 1)

	# wait
	robot.sleep(0.5)
	print(robot.pose())
	# go up
	robot.lmove(rel=0, z=z_up)  

	# trash
	#robot.jmove(rel= 0, j0=30, vel=200)

	# open gripper
	robot.output(0, 0)


if __name__ == '__main__':
	# variables
	num_a = 8
	num_b = 12

	# dorna object
	robot = Dorna()
	robot.connect("192.168.254.35")
	
	"""
	# user data
	print("Train the center, then X and finally Y")
	lattice_train = []
	for i in range(3):
		input(f"Press enter to capture cell {i}")
		print(robot.pose())
		lattice_train.append(np.array(robot.pose()))
	"""
	lattice_train = [np.array([ 2.60826424e+02, -8.30522720e+01, -2.88712050e+01, -9.12602500e+01,
        5.65987500e+01,  1.00000000e-01,  0.00000000e+00,  0.00000000e+00]), np.array([ 3.24189677e+02, -8.12052870e+01, -2.91333510e+01, -8.85872500e+01,
        5.49787500e+01,  1.00000000e-01,  0.00000000e+00,  0.00000000e+00]), np.array([ 2.57710527e+02,  1.63407800e+01, -2.85198720e+01, -9.11117500e+01,
        6.37987500e+01,  9.50000000e-02,  0.00000000e+00,  0.00000000e+00])]
	
	print(lattice_train)
	input(f"Press enter for operation")
	O = lattice_train[0]  
	A = lattice_train[1]
	B = lattice_train[2]

	# define lattice
	l = lattice(O, A, B, num_a, num_b)
	
	
	"""
	# input points
	points = [l.point(0, 0)]
	for point in points:
		pick_drop(robot, point)
	"""
	
	for i in range(num_a):
		for j in range(num_b):
			point = l.point(i, j)
			pick_drop(robot, point)		 
	
	# close dorna
	robot.close()
