from __future__ import print_function
import math

"""
all dimensions are in mm
l = [
		[95.53023, 0, 200.09099],
		[203.2, 0, 0],
		[152.4, 0, 0],
		[0, 0, 0],
		[0,  0, 48.92446]
	]	
"""
class kinematic(object):
	"""docstring for kinematic"""
	def __init__(self, 
		l = [
		[95.475806, 0, 206.404464],
		[203.2, 0, 0],
		[152.4, 0, 0],
		[0, 0, 0],
		[0,  0, 48.9245]]		
	):
		super(kinematic, self).__init__()
		# l matrix
		self.l = l
		self.e = 0.0001
	

	"""
	forward kinematics: joint to xyz
	"""
	def forward(self, joint):
		# joint to radian
		joint = [math.radians(j) for j in joint]

		# first we find x, y, z assuming base rotation is zero (j_0 = 0). Then we rotate everything
		# then we rotate the robot around z axis for j_0
		a = math.degrees(joint[1] + joint[2] + joint[3])
		b = math.degrees(joint[4])
		tmp_d = self.l[0][0] + self.l[1][0] * math.cos(joint[1]) + self.l[2][0] * math.cos(joint[1] + joint[2]) + self.l[4][2] * math.cos(joint[1] + joint[2] + joint[3])
		x = tmp_d * math.cos(joint[0])
		y = tmp_d * math.sin(joint[0])
		z = self.l[0][2] + self.l[1][0] * math.sin(joint[1]) + self.l[2][0] * math.sin(joint[1] + joint[2]) + self.l[4][2] * math.sin(joint[1] + joint[2] + joint[3])

		return [x, y, z, a, b]


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
		xa = self.l[0][0] + self.l[4][2] * math.cos(a)
		za = self.l[0][2] + self.l[4][2] * math.sin(a)

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
				if L >= self.l[1][0] + self.l[2][0] and L < self.l[1][0] + self.l[2][0] + self.e:
					theta_l1_L = 0  # l1 angle to L
					theta_l1_l2 = math.pi  # l1 angle to l2

				elif L <= self.l[1][0] - self.l[2][0] and L > self.l[1][0] - self.l[2][0] - self.e:
					theta_l1_L = 0  # l1 angle to L
					theta_l1_l2 = 0  # l1 angle to l2

				elif L < self.l[1][0] + self.l[2][0] and L > self.l[1][0] - self.l[2][0]:
					theta_l1_L = math.acos((self.l[1][0] ** 2 + L ** 2 - self.l[2][0] ** 2) / (2 * self.l[1][0] * L))  # l1 angle to L
					theta_l1_l2 = math.acos((self.l[1][0] ** 2 + self.l[2][0] ** 2 - L ** 2) / (2 * self.l[1][0] * self.l[2][0]))  # l1 angle to l2

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

		return joint


if __name__ == '__main__':
	k = kinematic()
	joint = [0, 150, 150, 150, 0]
	xyz = k.forward(joint)
	print(xyz)