import numpy as np
import time
import math
import random


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
		self.sum_alpha = np.pi/2 
		self.delta = np.pi/2 if ndof==5 else np.pi  

		self.ssa = math.sin(self.sum_alpha)
		self.cd = math.cos(self.delta)
		self.sd = math.sin(self.delta)


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
			ca = math.cos(ABC[0])
			sa = math.sin(ABC[0])

			cb = math.cos(ABC[1])
			sb = math.sin(ABC[1])

			cg = math.cos(ABC[2])
			sg = math.sin(ABC[2])

			ssa = self.ssa
			cd = self.cd
			sd = self.sd
			self.local_matrix =  np.matrix([
				[sa*sb*(cd*ssa*sg+cg*sd)+cb*(cg*cd-ssa*sg*sd),
				cg*(-cd*sb+cb*sa*sd)+ssa*sg*(cb*cd*sa+sb*sd),
				ca*(cd*ssa*sg+cg*sd), self.local_matrix[0,3]],
				[cg*ssa*(-cd*sa*sb+cb*sd)+sg*(cb*cd+sa*sb*sd),
				-sb*(cd*sg+cg*ssa*sd)+cb*sa*(-cg*cd*ssa+sg*sd),
				ca*(-cg*cd*ssa+sg*sd), self.local_matrix[1,3]],
				[ca*ssa*sb,
				ca*cb*ssa,
				-ssa*sa,self.local_matrix[2,3]],
				[0,0,0,1]])


	def get_euler(self,mode='DORNA'):
		if mode =='DORNA':

			thr = 0.0001

			sgn = 1.0

			ssa = self.ssa
			cd = self.cd
			sd = self.sd
			rot = self.local_matrix

			sa = -rot[2,2]*ssa
			ca = sgn * math.sqrt(1.0 - sa*sa)

			A = math.atan2(sa,ca)
			B = 0.0
			C = 0.0


			if abs(ca)>thr:

				sb = ssa*rot[2,0]/ca
				cb = ssa*rot[2,1]/ca
				B = math.atan2(sb,cb)

				sc =(rot[0,2]*cd*ssa+rot[1,2]*sd)/ca
				cc = (sd*rot[0,2]-rot[1,2]*cd*ssa) /ca
				
				C = math.atan2(sc,cc)

			else:

				sb = - ssa*(rot[1,0]*cd*sa + rot[1,1]*sd)
				cb = ssa*(-rot[1,1]*cd*sa + rot[1,0]*sd)

				B = math.atan2(sb,cb)

			return [A, B, C]

	def set_quaternion(self,quad):

		pass


	def get_quaternion(self,quad):

		pass