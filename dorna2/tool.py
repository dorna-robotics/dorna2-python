from __future__ import print_function
import math
import numpy as np

"""
All the dimensions are in mm

Transformation matrix between two coordinate systems
O is the original coordinate system
C is the new coordinate system

Case 1
	C rotated theta degree in x direction of O
	Then the transformation matrix M is the C in O coordinate system, [[xC in O], [yC in O], [zC in O]]
	PO = PC * M

Case 2
	if L = C - O
	then PO = l + PC * M, calculated based on case 1, and when C is shifted to O

"""
class trans(object):
	"""docstring for trans"""
	def __init__(self):
		super(trans, self).__init__()

	def rot_x(self, theta):
		t = math.radians(theta)
		cos = math.cos(t)
		sin = math.sin(t)
		return np.matrix([[1, 0, 0], [0, cos, sin], [0, -sin, cos]])			

	def rot_y(self, theta):
		t = math.radians(theta)
		cos = math.cos(t)
		sin = math.sin(t)
		return np.matrix([[cos, 0, -sin], [0, 1, 0], [sin, 0, cos]])			

	def rot_z(self, theta):
		t = math.radians(theta)
		cos = math.cos(t)
		sin = math.sin(t)
		return np.matrix([[cos, sin, 0], [-sin, cos, 0], [0, 0, 1]])		

"""
define the robot linkage relationship
				y	         y             y             x
	....l0...C1|__x...l1...C2|__x...l2...C3|__x...l3...C4|__z
	.........
	z
O,C0|__x
"""
class link(trans):
	"""docstring for link"""
	def __init__(self, 
		l = [
		[95.475806, 0, 206.404464],
		[203.2, 0, 0],
		[152.4, 0, 0],
		[48.9245, 0,  0]]):		
		super(link, self).__init__()
		self.l = np.matrix(l)

	"""
	find po, given pc0 and theta0
	"""
	def po_c0(self, p, theta, **arg):
		M = np.matrix(p) * self.rot_z(theta)
		return np.asarray(M).reshape(-1)

	def pc0_o(self, p, theta, **arg):
		M= np.matrix(p) * self.rot_z(-theta)
		return np.asarray(M).reshape(-1)

	"""
	find pc0, given pc1, theta1 and the position of c1 in c0: l = (l00, l01, l02) 
	"""
	def pc0_c1(self, p, theta, **arg):
		M = self.l[0] + np.matrix(p) * self.rot_z(theta) * self.rot_x(90)
		return np.asarray(M).reshape(-1)

	def pc1_c0(self, p, theta, **arg):
		M = (np.matrix(p) - self.l[0]) * self.rot_x(-90) * self.rot_z(-theta)
		return np.asarray(M).reshape(-1)

	"""
	find pc1, given pc2, theta2 and the position of c2 in c1: l = (l10, l11, l12) 
	"""
	def pc1_c2(self, p, theta, **arg):
		M = self.l[1] + np.matrix(p) * self.rot_z(theta)
		return np.asarray(M).reshape(-1)

	def pc2_c1(self, p, theta, **arg):
		M = (np.matrix(p) - self.l[1]) * self.rot_z(-theta)
		return np.asarray(M).reshape(-1)

	"""
	find pc2, given pc3, theta3 and the position of c3 in c2: l = (l20, l21, l22) 
	"""
	def pc2_c3(self, p, theta, **arg):
		M = self.l[2] + np.matrix(p) * self.rot_z(theta)
		return np.asarray(M).reshape(-1)

	def pc3_c2(self, p, theta, **arg):
		M = (np.matrix(p) - self.l[2]) * self.rot_z(-theta)
		return np.asarray(M).reshape(-1)

	"""
	find pc3, given pc4, theta4 and the position of c4 in c3: l = (l30, l31, l32) 
	"""
	def pc3_c4(self, p, theta, **arg):
		M = self.l[3] + np.matrix(p) * self.rot_z(theta) * self.rot_z(-90) * self.rot_y(-90)
		return np.asarray(M).reshape(-1)

	def pc4_c3(self, p, theta, **arg):
		M = (np.matrix(p) - self.l[3]) * self.rot_y(90) * self.rot_z(90) * self.rot_z(-theta)
		return np.asarray(M).reshape(-1)


class kinematic(link):
	"""docstring for kinematic"""
	def __init__(self, e=0.0001):
		super(kinematic, self).__init__()
		self.e = e
	
	"""
	forward kinematics: joint to xyz
	"""
	def forward(self, joint):
		a = joint[1] + joint[2] + joint[3]
		b = joint[4]	
			
		p4 = [0, 0, 0]
		p3 = self.pc3_c4(p4, joint[4])
		p2 = self.pc2_c3(p3, joint[3])
		p1 = self.pc1_c2(p2, joint[2])
		p0 = self.pc0_c1(p1, joint[1])
		po = self.po_c0(p0, joint[0])
		return np.append(po, [a, b])

	"""
	return degree from (-180, 180]
	"""
	def adjust_degree(self, ang):
		ang = ang % 360
		if ang > 180:
			ang = ang - 360
		return ang

	"""
	inverse kinematics: xyz to joint
	"""
	def inverse(self, xyz):
		x = xyz[0]
		y = xyz[1]
		z = xyz[2]
		a = math.radians(xyz[3])
		b = math.radians(xyz[4])
		
		joint = []	
		
		# first we find the base rotation
		theta_0 = math.atan2(y, x)

		# x
		xy = math.sqrt(x ** 2 + y ** 2)

		# xa and za
		xa = self.l[0, 0] + self.l[3, 0] * math.cos(a)
		za = self.l[0, 2] + self.l[3, 0] * math.sin(a)

		for i in range(2):
			try:
				# j0
				j0 = theta_0 + i * math.pi

				# x, z
				x = (-2*i + 1) * xy - xa
				z = xyz[2] - za
				# at this point x and z are the summation of two vectors one from lower arm and one from upper arm of lengths l1 and l2
				# let L be the length of the overall vector
				# we can calculate the angle between l1 , l2 and L
				L = math.sqrt(x ** 2 + z ** 2)
				if L >= self.l[1, 0] + self.l[2, 0] and L < self.l[1, 0] + self.l[2, 0] + self.e:
					theta_l1_L = 0  # l1 angle to L
					theta_l1_l2 = math.pi  # l1 angle to l2

				elif L <= self.l[1, 0] - self.l[2, 0] and L > self.l[1, 0] - self.l[2, 0] - self.e:
					theta_l1_L = 0  # l1 angle to L
					theta_l1_l2 = 0  # l1 angle to l2

				elif L < self.l[1, 0] + self.l[2, 0] and L > self.l[1, 0] - self.l[2, 0]:
					theta_l1_L = math.acos((self.l[1, 0] ** 2 + L ** 2 - self.l[2, 0] ** 2) / (2 * self.l[1, 0] * L))  # l1 angle to L
					theta_l1_l2 = math.acos((self.l[1, 0] ** 2 + self.l[2, 0] ** 2 - L ** 2) / (2 * self.l[1, 0] * self.l[2, 0]))  # l1 angle to l2

				else:
					continue 

				theta_L_x = math.atan2(z, x)  # L angle to x axis
				
				for j in range(2):
					j1 = theta_L_x + (-2*j + 1) * theta_l1_L
					j2 = (-2*j + 1)* (theta_l1_l2 - math.pi)
					j3 = a- j1 - j2
		
					joint.append([
						self.adjust_degree(math.degrees(j0)), 
						self.adjust_degree(math.degrees(j1)), 
						self.adjust_degree(math.degrees(j2)), 
						self.adjust_degree(math.degrees(j3)), 
						b
					])
			except:
				pass

		return np.matrix(joint)

if __name__ == '__main__':
	k = kinematic()
	joint = [61, 11, 75, -30, 0]
	xyz = k.forward(joint)
	print("joint: ", joint)
	print("forward: ", xyz)
	print("inverse: ", k.inverse(xyz))


