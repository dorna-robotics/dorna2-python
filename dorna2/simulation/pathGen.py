import numpy as np
from dorna2 import Dorna



class Path:
	def __init__(self, points):
		self.num_steps = len(points)
		self.points = points

	def get_point(self, t):
		n = t * (self.num_steps - 1)
		
		if np.isclose(n, np.round(n)):
			return self.points[int(np.round(n))]

		n1 = int(np.floor(n))
		n2 = n1 + 1

		return self.mix_points(self.points[n1] , self.points[n2], n - n1)

	def mix_points(self, p1, p2, t):
		p1 = np.array(p1)
		p2 = np.array(p2)
		res = p1 + (p2 - p1) * t
		return res.tolist()


class pathGen:

	def __init__(self, motion_, j1_, j2_, steps_, kinematic_, tcp_):
		self.kin = kinematic_
		
		if steps_<3:
			print("number of steps is smaller that 3")
			return

		if motion_!="lmove" and motion_!="jmove":
			print("motion type should be either lmove or jmove")
			return

		self.steps = steps_
		self.motion = motion_
		
		self.singular = False

		self.velocity_treshold = 100

		if self.motion=="jmove":
			self.path = self.gen_jmove(j1_,j2_,steps_)

		if self.motion=="lmove":
			self.path = self.gen_lmove(j1_,j2_,steps_,tcp_)

	def gen_jmove(self, j1, j2, steps):
		j1 = np.array(j1)
		j2 = np.array(j2)

		points = []

		for i in range(steps):
			t = float(i)/float(steps-1)
			points.append((j1+(j2-j1)*t).tolist())

		return Path(points)

	def gen_lmove(self, j1, j2, steps, tcp):
		self.kin.set_frame_tcp(frame=None, tcp=self.kin.xyzabc_to_mat(tcp))
		fw1 = np.array(self.kin.fw(j1))
		fw2 = np.array(self.kin.fw(j2))
		jj1 = np.array(j1)
		jj2 = np.array(j2)

		points = [np.array(j1).tolist()]
		last_joint = j1

		for i in range(1, steps):
			t = float(i)/float(steps-1)

			fw = fw1 + (fw2-fw1) * t
			jj = jj1 + (jj2-jj1) * t
			ikj = self.kin.ik_xyzj345([fw[0],fw[1],fw[2]] , [float(jj[3]),float(jj[4]),float(jj[5])], tcp, last_joint)

			new_joint = jj
			new_joint[:3]= ikj[:3]
			print(last_joint[:3])
			velocity = np.linalg.norm(np.array(new_joint) - np.array(last_joint))
			if velocity > self.velocity_treshold : 
				self.singular = True

			last_joint = new_joint
			points.append(last_joint)

		return Path(points)