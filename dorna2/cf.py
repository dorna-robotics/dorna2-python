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