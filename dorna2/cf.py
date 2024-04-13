import numpy as np
import time
import math
import random

def clamp(num, min_value, max_value):
        #num = max(min(num, max_value), min_value)
        return num
#Defining the class for Coordinate frames
class CF(object): 
	def __init__(self,parent_CF = None,local_matrix = None, ndof = 5):
		super(CF, self).__init__()
		self.parent_CF = parent_CF				#Parent coordinate frame pointer , CF type
		self.global_matrix = np.identity(4)		#Transform matrix with respect to global coordinate frame

		if(local_matrix is None):
			self.local_matrix = np.identity(4)
		else:
			self.local_matrix = local_matrix		#Transform matrix with respect to parent coordinate frame

		#self.local_speed = np.array([0,0,0])
		#self.global_speed = np.array([0,0,0])
		
		self.children = []						#Children coordinate frames

		self._update()

		#robot dependant vairables
		self.ndof = ndof

	#Evaluates global matrix for self and all children, everytime local matrix has changed, 
	def _update(self): 
		#update global matrix
		if self.parent_CF is None:
			self.global_matrix =  self.local_matrix
		else:
			self.global_matrix = np.matmul(self.parent_CF.global_matrix, self.local_matrix) 

		#call update function on all children
		for child in self.children:
			child._update()


	def set_parent(self,parent):
		self.parent_CF = parent
		self._update()

	def set_matrix(self,mat):
		self.local_matrix = mat
		self._update()

	def set_pos(self,vec):
		self.mat[0,4] = vec[0]
		self.mat[1,4] = vec[1]
		self.mat[2,4] = vec[2]
		self.mat[3,4] = vec[3]
		self._update()

	def set_euler(self,ABC,mode='DORNA'):
		if mode == 'DORNA':
			c1 = math.cos(ABC[0])
			s1 = math.sin(ABC[0])

			c2 = math.cos(ABC[1])
			s2 = math.sin(ABC[1])

			c3 = math.cos(ABC[2])
			s3 = math.sin(ABC[2])


			self.local_matrix =  np.matrix([
				[c2*c1 , s3*s2*c1 - c3*s1 , c3*s2*c1 + s3*s1, self.local_matrix[0,3]],
				[c2*s1 , s3*s2*s1 + c3*c1 , c3*s2*s1 - s3*c1, self.local_matrix[1,3]],
				[-s2 , s3*c2 , c3*c2,self.local_matrix[2,3]],
				[0,0,0,1]])


	def get_euler(self,mode='DORNA'):
		#if mode =='DORNA':
		rot = self.local_matrix

		a=0
		b=0
		c=0

		if(1.-abs(rot[2,0])>0.0001):
			b = -np.arcsin(rot[2,0])

			c = np.arctan2(rot[2,1]/np.cos(b) , rot[2,2]/np.cos(b))

			a = np.arctan2(rot[1,0]/np.cos(b) , rot[0,0]/np.cos(b))
		else:
			a = 0
			if rot[2,0]<0 :
				b = np.pi/2
				c = a + np.arctan2(rot[0,1],rot[0,2])
			
			else:
				b = -np.pi/2
				c = -a + np.arctan2(-rot[0,1],-rot[0,2])				

		
		return [a,b,c];


	def set_quaternion(self,quad):

		pass


	def get_quaternion(self,quad):

		pass

	def mat_to_quat(self,mat):
		m00 = mat[0,0]
		m10 = mat[1,0]
		m20 = mat[2,0]

		m01 = mat[0,1]
		m11 = mat[1,1]
		m21 = mat[2,1]

		m02 = mat[0,2]
		m12 = mat[1,2]
		m22 = mat[2,2]


		tr = m00 + m11 + m22

		qw = 0
		qz = 0
		qx = 0
		qy = 0

		if tr > 0:
			S = math.sqrt(tr+1.0) * 2 
			qw = 0.25 * S
			qx = (m21 - m12) / S
			qy = (m02 - m20) / S
			qz = (m10 - m01) / S

		elif (m00 > m11) and (m00 > m22):
			S = math.sqrt(1.0 + m00 - m11 - m22) * 2 
			qw = (m21 - m12) / S
			qx = 0.25 * S
			qy = (m01 + m10) / S
			qz = (m02 + m20) / S

		elif m11 > m22:
			S = math.sqrt(1.0 + m11 - m00 - m22) * 2
			qw = (m02 - m20) / S
			qx = (m01 + m10) / S
			qy = 0.25 * S
			qz = (m12 + m21) / S

		else:
			S = math.sqrt(1.0 + m22 - m00 - m11) * 2
			qw = (m10 - m01) / S
			qx = (m02 + m20) / S
			qy = (m12 + m21) / S
			qz = 0.25 * S

		return [qx,qy,qz,qw]

	def quat_to_mat(self,quat):
		qx, qy, qz , qw = (quat[0], quat[1], quat[2], quat[3])
		qy2 = qy*qy
		qz2 = qz*qz
		qx2 = qx*qx
		return np.matrix([[1 - 2*qy2 - 2*qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
						[2*qx*qy + 2*qz*qw, 1 - 2*qx2 - 2*qz2, 2*qy*qz - 2*qx*qw],
						[2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx2 - 2*qy2]])

	def quat_xyz_to_mat(self,quat,xyz):
		qx, qy, qz , qw = (quat[0], quat[1], quat[2], quat[3])
		qy2 = qy*qy
		qz2 = qz*qz
		qx2 = qx*qx
		return np.matrix([[1 - 2*qy2 - 2*qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, xyz[0]],
						[2*qx*qy + 2*qz*qw, 1 - 2*qx2 - 2*qz2, 2*qy*qz - 2*qx*qw, xyz[1]],
						[2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx2 - 2*qy2, xyz[2]],
						[0,0,0,1]])

	def quat_dot_product(self, q1, q2):
		return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]

	def quat_slerp(self, q1, q2 , t):
		#t goes between [0,1]
		dot_product = self.quat_dot_product(q1,q2)

		theta = np.arccos(clamp(dot_product,-1,1) )

		sin1 = np.sin((1 - t) * theta);
		sin2 = np.sin(t * theta);

		result = [0,0,0,0]

		if(abs(np.sin(theta))>1e-5):
			result[0] = (sin1 * q1[0] + sin2 * q2[0]) / np.sin(theta)
			result[1] = (sin1 * q1[1] + sin2 * q2[1]) / np.sin(theta)
			result[2] = (sin1 * q1[2] + sin2 * q2[2]) / np.sin(theta)
			result[3] = (sin1 * q1[3] + sin2 * q2[3]) / np.sin(theta)
		else:
			result = q1
			
		return result

	def quat_mult(self, q1, q2):
		w1, x1, y1, z1 = (q1[0], q1[1], q1[2], q1[3])
		w2, x2, y2, z2 = (q2[0], q2[1], q2[2], q2[3])

		w = w1*w2 - x1*x2 - y1*y2 - z1*z2
		x = w1*x2 + x1*w2 + y1*z2 - z1*y2
		y = w1*y2 - x1*z2 + y1*w2 + z1*x2
		z = w1*z2 + x1*y2 - y1*x2 + z1*w2

		return [x,y,z,w]


	def xyzabc_to_mat(self,xyzabc):
		c1 = math.cos(xyzabc[3])
		s1 = math.sin(xyzabc[3])

		c2 = math.cos(xyzabc[4])
		s2 = math.sin(xyzabc[4])

		c3 = math.cos(xyzabc[5])
		s3 = math.sin(xyzabc[5])


		return  np.matrix([
			[c2*c1 , s3*s2*c1 - c3*s1 , c3*s2*c1 + s3*s1, xyzabc[0]],
			[c2*s1 , s3*s2*s1 + c3*c1 , c3*s2*s1 - s3*c1, xyzabc[1]],
			[-s2 , s3*c2 , c3*c2,						  xyzabc[2]],
			[0,0,0,1]])