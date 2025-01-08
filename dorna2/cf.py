import numpy as np
import math

def clamp(num, min_value, max_value):
        num = max(min(num, max_value), min_value)
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

	def set_euler(self,ABC):
		self.local_matrix =  self.xyzabc_to_mat([self.local_matrix[0,3], self.local_matrix[1,3], self.local_matrix[2,3],
			ABC[0] , ABC[1], ABC[2] ]) 


	def get_euler(self):
		xyzabc = self.mat_to_xyzabc(self.local_matrix)
		
		return [xyzabc[3], xyzabc[4], xyzabc[5]]


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


	def xyzabc_to_mat(self, xyzabc):

		abc = [xyzabc[3], xyzabc[4], xyzabc[5]]
		mat = self.axis_angle_to_mat(abc)
		mat[0,3] = xyzabc[0]
		mat[1,3] = xyzabc[1]
		mat[2,3] = xyzabc[2]
		return mat

	def mat_to_xyzabc(self, matrix):
		#ZY'Z'' convention
		abc = self.mat_to_axis_angle(matrix)

		xyzabc = [matrix[0,3], matrix[1,3], matrix[2,3], abc[0], abc[1], abc[2]]

		return np.array(xyzabc)


	def mat_to_axis_angle(self,mat):

		trace =	mat[0,0] + 	mat[1,1] +	mat[2,2]

		theta = np.arccos(clamp((trace - 1.) / 2.,-1.,1.) );


		st = np.sin(theta);
		ct = np.cos(theta);
		
		u = [0,0,0]

		if (theta < 0.0000001):
			return u
		
		if (np.pi - theta > 0.000001):
			u[2] = (mat[1,0] - mat[0,1]) / st /2.
			u[1] = (mat[0,2] - mat[2,0]) / st / 2.
			u[0] = (mat[2,1] - mat[1,2]) / st / 2.
		else:
			theta = np.pi
			u[0] = np.sqrt(max(mat[0, 0]+ 1.,0.) / 2.);
			u[1] = np.sqrt(max(mat[1, 1]+ 1.,0.) / 2.);
			u[2] = np.sqrt(max(mat[2, 2]+ 1.,0.) / 2.);

			c1 = mat[1, 0]
			c2 = mat[2, 0]

			if (c1 < 0.):
				u[1] = -u[1]
			
			if (c2 < 0.) :
				u[2] = -u[2]
		
		theta = theta * 180 / np.pi

		return [u[0]*theta , u[1]*theta, u[2]*theta]


	def axis_angle_to_mat(self, u):

		lu = np.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])

		theta = np.pi / 180 * lu

		ct = np.cos(theta)
		st = np.sin(theta)

		if lu>0.00001:
			u[0] /= lu
			u[1] /= lu
			u[2] /= lu

		mat = [
		[ct + u[0]*u[0]*(1.-ct) , u[0]*u[1]*(1-ct) - u[2]*st, u[0]*u[2]*(1.-ct) + u[1]*st , 0],
		[u[1]*u[0]*(1.-ct) + u[2]*st , ct + u[1]*u[1]*(1.-ct) , u[1]*u[2]*(1.-ct) - u[0]*st, 0] ,
		[u[2]*u[0]*(1.-ct) - u[1]*st , u[2]*u[1]*(1.-ct)+u[0]*st , ct + u[2]*u[2]*(1.-ct),0],
		[0,0,0,1]
		]

		return np.matrix(mat)

	def get_X_axis(self, mat = None, xyzabc = None):

		if xyzabc is not None:
			mat = self.xyzabc_to_mat(xyzabc)
		if mat is None:
			return np.array([1.0,0.0,0.0])
		x = np.array([0.0,0.0,0.0])
		x[0] = mat[0,0]
		x[1] = mat[1,0]
		x[2] = mat[2,0]
		return x

	def get_Y_axis(self, mat = None, xyzabc = None):

		if xyzabc is not None:
			mat = self.xyzabc_to_mat(xyzabc)
		if mat is None:
			return np.array([0.0,1.0,0.0])

		y = np.array([0.0,0.0,0.0])
		y[0] = mat[0,1]
		y[1] = mat[1,1]
		y[2] = mat[2,1]

		return y

	def get_Z_axis(self, mat = None, xyzabc = None):
		if xyzabc is not None :
			mat = self.xyzabc_to_mat(xyzabc)
		if mat is None :
			return np.array([0.0,0.0,1.0])

		z = np.array([0.0,0.0,0.0])
		z[0] = mat[0,2]
		z[1] = mat[1,2]
		z[2] = mat[2,2]

		return z

	def get_Z_axis(self, mat = None, xyzabc = None):
		if xyzabc is not None :
			mat = self.xyzabc_to_mat(xyzabc)
		if mat is None :
			return np.array([0.0,0.0,1.0])

		z = np.array([0.0,0.0,0.0])
		z[0] = mat[0,2]
		z[1] = mat[1,2]
		z[2] = mat[2,2]

		return z

	def rotate_rvec(self, rvec=[0,0,0], axis=[1,0,0], angle=0, local=False ):
		
		axis = np.array(axis)

		T = self.axis_angle_to_mat([rvec[0],rvec[1],rvec[2]])

		if local:
			axis = axis[0] * self.get_X_axis(T) + axis[1] * self.get_Y_axis(T) + axis[2] * self.get_Z_axis(T)

		axis = axis / np.linalg.norm(axis) 

		R = self.axis_angle_to_mat(axis * angle)

		RT = np.matmul(R,T)

		RTv = self.mat_to_axis_angle(RT)

		return [RTv[0], RTv[1], RTv[2]]
		 


	def xyzabc_to_xyzquat(self, xyzabc):
		xyz = np.array([xyzabc[0],xyzabc[1],xyzabc[2]]) / 1000
		quat = self.mat_to_quat(self.xyzabc_to_mat(xyzabc))

		return [xyz[0], xyz[1], xyz[2],  quat[3], quat[0], quat[1], quat[2]]

	def xyzquat_to_xyzabc(self, xyzquat):
		xyz = np.array([xyzquat[0], xyzquat[1], xyzquat[2]]) * 1000
		quat = np.array([ xyzquat[4], xyzquat[5] , xyzquat[6], xyzquat[3]])
		mat3 = np.matrix(self.quat_to_mat(quat))
		mat4 = np.eye(4)
		mat4[:3,:3] = mat3
		xyzabc = self.mat_to_xyzabc(mat4)
		xyzabc[:3] = xyz[:3]

		return xyzabc

