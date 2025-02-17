import numpy as np
from dorna2.ik6r_2 import ik

from dorna2.cf import CF
#from cf import CF
"""
Sources:
	http://rasmusan.blog.aau.dk/files/ur5_kinematics.pdf
	https://smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf

i | alpha[i-1]        | a[i-1] | d[i] | theta[i]
------------------------------------------------
1 | 0                 | 0      | d[1] | 
2 | alpha[1]= np.pi/2 | a[1]   | 0    | 
3 | 0                 | a[2]   | 0    |
4 | 0                 | a[3]   | d[4] |
5 | alpha[4]= np.pi/2 | 0      | d[5] |
6 | alpha[5]= np.pi/2 | 0      | d[6] |

"""

def clamp(num, min_value, max_value):
        #num = max(min(num, max_value), min_value)
        return num

def d_theta(t1,t2):
	dt = t2 - t1
	while dt > np.pi:
		dt = dt - 2 * np.pi
	while dt < -np.pi:
		dt = dt + 2 * np.pi
	return dt

def angle_space_distance(s1, s2):
    return np.linalg.norm(np.array(s1)[0:min(len(s1), len(s2))] - np.array(s2)[0:min(len(s1), len(s2))])


def dot(a,b):
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

class DH(CF):
	"""docstring for dh"""
	def __init__(self):
		super(DH, self).__init__()

		self.n_dof = 6 # number of degrees of freedom, choose between [5,6]
		self.alpha = [0, 0, np.pi/2, 0, 0, 0, 0] # Rotation of Ci with respect to C(i-1) around the x axis of Ci
		self.delta = [0, 0, 0, 0, 0, np.pi/2, np.pi/2] #Rotation of Ci with respect to C(i-1) around the y axis of Ci
		self.a = [0, 1, 1, 1, 0,0,0] # Translation of Ci with respect to C(i-1) along the x axis of Ci
		self.d = [0, 1, 0, 0,-1,1,1] #Translation of Ci with respect to C(i-1) along the z axis of Ci
		self.rail_vec_r_base = [0,0,0]  #The vector that describes the motion of base on rail
		self.rail_limit = [-100,200] #max and min value for rail joint
		self.rail_on = False


		self.set_frame_tcp(frame = np.identity(4), tcp = np.identity(4))

	def set_frame_tcp(self, frame, tcp):
		if frame is not None:
			self.T_rail_r_world =  frame# base r world
			self.inv_T_rail_r_world =  np.linalg.inv(self.T_rail_r_world)# base r world
		if tcp is not None:
			self.T_tcp_r_flange = tcp # TCP r flange
			self.inv_T_tcp_r_flange = np.linalg.inv(self.T_tcp_r_flange) # TCP r flange		

	def set_tcp_xyzabc(self, xyzabc):
		abc = [xyzabc[3], xyzabc[4], xyzabc[5]]
		tcp = self.axis_angle_to_mat(abc)
		tcp[0,3] = xyzabc[0]
		tcp[1,3] = xyzabc[1]
		tcp[2,3] = xyzabc[2]
		self.set_frame_tcp( frame = None, tcp = tcp)

	def T(self, i, theta):
		ct = np.cos(theta+self.delta[i])
		st = np.sin(theta+self.delta[i])
		ca = np.cos(self.alpha[i])
		sa = np.sin(self.alpha[i])
		cd = np.cos(self.delta[i])
		sd = np.sin(self.delta[i])
		ai = self.a[i]
		di = self.d[i]
		result =  np.matrix([
			[ct,-st*ca,st*sa,ai*ct],
			[st,ct*ca,-ct*sa,ai*st],
			[0,sa,ca,di],
			[0, 0, 0, 1]])

		return result

	def dT(self, i, theta):
		ct = np.cos(theta)
		st = np.sin(theta)
		ca = np.cos(self.alpha[i])
		sa = np.sin(self.alpha[i])
		cd = np.cos(self.delta[i])
		sd = np.sin(self.delta[i])

		result =  np.matrix([
			[-cd*st, -cd*ct, 0, 	0],
			[-st*sa*sd+ca*ct, -ca*st+sa*sd*ct, 0,	0],
			[+ ca*st*sd + sa*ct, -st*sa+ca*sd*ct, 0,	0],
			[0, 0, 0, 0]])

		return result


	def inv_dh(self, T):
		R = np.matrix([
			[T[0,0], T[1,0], T[2,0]],
			[T[0,1], T[1,1], T[2,1]],
			[T[0,2], T[1,2], T[2,2]]
		])
		S = -np.matmul(R, [[T[0,3]], [T[1,3]], [T[2,3]]])

		return np.matrix([
			[R[0,0], R[0,1], R[0,2], S[0,0]],
			[R[1,0], R[1,1], R[1,2], S[1,0]],
			[R[2,0], R[2,1], R[2,2], S[2,0]],
			[0, 0, 0, 1]	
		])

class Dof(DH):
	"""docstring for dof"""
	def __init__(self):
		super(Dof, self).__init__()
		self.thr = 0.0001

	def t_flange_r_world(self, theta=None , joint = None):

		if joint is not None:
			theta = list(joint)
			theta =  [np.radians(j) for j in theta]

		T = self.T_rail_r_world.copy()

		if self.n_dof == 5:#rail transformation
			rail_d_vec = np.matmul(T, [[self.rail_vec_r_base[0]*theta[5]], [self.rail_vec_r_base[1]*theta[5]], [self.rail_vec_r_base[2]*theta[5]], [0]])
			T[0,3] += rail_d_vec[0,0]
			T[1,3] += rail_d_vec[1,0]
			T[2,3] += rail_d_vec[2,0]

		for i in range(0, self.n_dof+1):
			if i==0:
				T = np.matmul(T, self.T(i, 0))
			else:
				T = np.matmul(T, self.T(i, theta[i-1]))

		return T

	def Ti_r_world(self, theta=None , joint = None , i = 0): #joint is in degrees, theta is in radians

		if joint is not None:
			theta = list(joint)
			theta =  [np.radians(j) for j in theta]

		T = np.eye(4)#self.T_rail_r_world.copy()


		for j in range(0, i+1):
			if j==0:
				T = np.matmul(T, self.T(j, 0))
			else:
				T = np.matmul(T, self.T(j, theta[j-1]))

		return T

	def jacobian_flange_r_world(self, theta=None , joint = None ):

		if joint is not None:
			theta = list(joint)
			theta =  [np.radians(j) for j in theta]

		res = []

		for j in range(1, self.n_dof+1):
			T = np.identity(4)
			for i in range(1, self.n_dof+1):
				if i != j:
					T = np.matmul(T, self.T(i, theta[i-1]))
				else:
					T = np.matmul(T, self.dT(i, theta[i-1]))

			res.append(T)
		return res

	def fw_base(self, theta):			
		T_flange_r_world = self.t_flange_r_world(theta)

		return np.matmul(T_flange_r_world, self.T_tcp_r_flange)

	"""
	The robot T_tcp_r_base is given
	find all the possible robot orientations 
	"""

	def inv_base(self, T_tcp_r_world, theta_current, all_sol, freedom ):

		goal_matrix = np.matmul(T_tcp_r_world, self.inv_T_tcp_r_flange)


		if all_sol:

			all_sol = ik(self.a[1],self.a[2],self.d[0],-self.d[3],self.d[4],self.d[5],self.d[6], goal_matrix)
			approved_sol = []
			for s in all_sol:
				t = self.t_flange_r_world(theta=s)
				mdist = np.sqrt(np.sum( np.array(t - goal_matrix)**2))
				if(mdist>0.01):
					continue

				approved_sol.append(s)
				
			return approved_sol

		if theta_current is not None: 

			initial_xyzabc = self.mat_to_xyzabc( self.t_flange_r_world(theta = theta_current))
			goal_xyzabc = self.mat_to_xyzabc( goal_matrix)

			joint_space_distance_treshold = np.sqrt(np.sum( np.array(initial_xyzabc - goal_xyzabc)**2)) / 50

			#print("treshold ",joint_space_distance_treshold)

			all_sol = []
			if freedom is None:
				new_sol = ik(self.a[1],self.a[2],self.d[0],-self.d[3],self.d[4],self.d[5],self.d[6], goal_matrix)
				for s in new_sol:
					t = self.t_flange_r_world(theta = s)
					mdist = np.sqrt(np.sum( np.array(t - goal_matrix)**2))
					if(mdist>0.01):
						continue
					all_sol.append(s)
			else:
				counter = 0
				for sample in range(max(1,freedom["num"])):

					tmp_matrix = goal_matrix
					

					if counter>0:
						random_vector = (np.random.rand(3) - np.array([0.5,0.5,0.5])) * 2

						if "range" in freedom:
							pass
						else:
							freedom["range"] = [4.0,4.0,4.0] #in degrees

						random_vector =   self.get_X_axis(tmp_matrix) * random_vector[0] * freedom["range"][0] + self.get_Y_axis(tmp_matrix) * random_vector[1] * freedom["range"][1] + self.get_Z_axis(tmp_matrix) * random_vector[2] *freedom["range"][2]

						random_rotation_matrix = self.axis_angle_to_mat(random_vector)
						tmp_matrix = np.matmul(tmp_matrix,  random_rotation_matrix)
						

					counter = counter + 1

					new_sol = ik(self.a[1],self.a[2],self.d[0],-self.d[3],self.d[4],self.d[5],self.d[6], tmp_matrix)

					no_need_to_continue = False

					for s in new_sol:
						t = self.t_flange_r_world(theta = s)
						mdist = np.sqrt(np.sum( np.array(t - tmp_matrix)**2))
						if(mdist>0.01):
							continue

						#if(counter == 1):#check if first try has valid solutions
						#	if angle_space_distance(np.array(s) , np.array(theta_current))<joint_space_distance_treshold:
						#		no_need_to_continue = True

						if("early_exit" in freedom ):
							if(freedom["early_exit"]):
								if angle_space_distance(np.array(s) , np.array(theta_current))<joint_space_distance_treshold:
									no_need_to_continue = True	

						all_sol.append(s)

					if no_need_to_continue:
						break

			#print("counter samples: ", len(all_sol))
			best_sol_dist = 10000
			best_sol = 0
			indx = 0

			num_of_recieved_solutions = 0

			for s in all_sol:

				num_of_recieved_solutions += 1

			
				dist = angle_space_distance(np.array(s) , np.array(theta_current))

				if dist < best_sol_dist:
					best_sol_dist = dist
					best_sol = s

			#print("best sol dist: ", best_sol_dist)
			if best_sol_dist < 5.0:
				return [best_sol]
			else:
				return[]

		return []

	def  nearest_pose(self, poses, current_joint, freedom):

		current_theta = np.array(current_joint)*np.pi/180.0

		if len(poses) < 1:
			return 0

		best_pose = poses[0]
		
		best_distance = 1000000

		for pose in poses:

			goal_matrix = self.xyzabc_to_mat(pose)

			ik_sols = self.inv_base(np.array(goal_matrix), theta_current = current_theta, all_sol = False , freedom = freedom)
			for s in ik_sols:

				dist = angle_space_distance(np.array(s) , np.array(current_theta))
				if dist < best_distance:
					best_distance = dist
					best_pose = pose

		return best_pose

	def approach(self, T_tcp_r_world, theta_current):
		try: 
			current_tcp = np.matrix(self.t_flange_r_world(theta = theta_current))
			i_current_tcp = np.linalg.inv(current_tcp)
			current_xyz = np.array([current_tcp[0,3], current_tcp[1,3], current_tcp[2,3]])
			#current_quat = np.array(self.mat_to_quat(current_tcp))
			current_aa = np.array(self.mat_to_axis_angle(current_tcp))

			goal_xyz = np.array([T_tcp_r_world[0,3], T_tcp_r_world[1,3], T_tcp_r_world[2,3]])
			#goal_quat = np.array(self.mat_to_quat(T_tcp_r_world))
			goal_aa = np.array(self.mat_to_axis_angle(T_tcp_r_world))

			max_approved_joint_distance = 1.0

			tcp = T_tcp_r_world

			for i in range(10): #at most go 4 step to find a good goal position

				#tcp = np.matmul(np.matmul(self.inv_T_rail_r_world, tcp), self.inv_T_tcp_r_flange)

				sol = ik(self.a[1],self.a[2],self.d[0],-self.d[3],self.d[4],self.d[5],self.d[6], tcp)
				
				#find nearest sol
				best_sol_dist = 10000
				best_sol = []
				for s in sol:

					t = self.fw_base(theta=s)
					mdist = np.sqrt(np.sum( np.array(t - tcp)**2))

					if(mdist>0.01):
						continue

					dist = angle_space_distance(np.array(s) , np.array(theta_current))
					if dist < best_sol_dist and (s[2])*(theta_current[2]-0.05)>0:
						best_sol_dist = dist
						best_sol = s

				if best_sol_dist < max_approved_joint_distance:
					return [best_sol]
				

				t =  max(max_approved_joint_distance / best_sol_dist ,  0.2)
				goal_xyz = current_xyz + (goal_xyz - current_xyz) * t
				#goal_quat = self.quat_slerp(current_quat, goal_quat, t)
				

				#goal_aa = current_aa + (goal_aa - current_aa)*t

				#tcp = self.quat_xyz_to_mat(goal_quat, goal_xyz)

				iCG = np.matmul(i_current_tcp, tcp)
				goal_aa = np.array(self.mat_to_axis_angle(iCG)) * t


				tcp = np.matmul(current_tcp, self.axis_angle_to_mat(goal_aa))

				tcp[0,3] = goal_xyz[0]
				tcp[1,3] = goal_xyz[1]
				tcp[2,3] = goal_xyz[2]

			return [theta_current]
		except:
			return [theta_current]

	def theta_1(self, T_last_r0, t1=None):
		if(self.n_dof == 6):
			p5_0 = np.matmul(T_last_r0, [[0], [0], [-self.d[6]], [1]])
		else:
			p5_0 = np.matmul(T_last_r0, [[0], [0], [0], [1]])

		p5x_0 = p5_0[0,0]
		p5y_0 = p5_0[1,0]
		rtn = []
		try:
			alpha = np.asin(clamp(-self.d[4]/np.sqrt(p5x_0**2 + p5y_0**2), -1.0 , 1.0))
			phi_1 = np.atan2(p5y_0, p5x_0)

			rtn = [phi_1-alpha, phi_1+alpha-np.pi]
			if t1 is not None:
				if min(abs(t1-rtn[0]%(2*np.pi)), abs(t1-rtn[0]%(-2*np.pi))) > min(abs(t1-rtn[1]%(2*np.pi)), abs(t1-rtn[1]%(-2*np.pi))):
					rtn.pop(0)
				else:
					rtn.pop(1)
			return rtn
		except Exception as ex:
			return rtn

	def theta_5(self, T_last_r0, theta_1, t5=None , abc=[0,0,0]):
		if self.n_dof == 5:
			return [abc[1]]

		nom = T_last_r0[1,3]*np.cos(theta_1)-T_last_r0[0,3]*np.sin(theta_1)+self.d[4]
		rtn = []
		try:
			phi = np.acos(clamp(nom/self.d[6],-1.0,1.0))
			rtn = [phi, -phi]
			
			if t5 is not None:
				if t5*rtn[0] >= 0:
					rtn.pop(1)
				else:
					rtn.pop(0)

			return rtn
		except Exception as ex:
			return []


	def theta_6(self, T_last_r0, theta_1, theta_5, theta_6_init=0):

		if(self.n_dof==5):
			return 0

		if abs(np.sin(theta_5)) < self.thr:
			return theta_6_init

		sgn_t_5 = 1.0
		if np.sin(theta_5)<0:
			sgn_t_5 = -1.0

		cos = sgn_t_5*(T_last_r0[0,1]*np.sin(theta_1) - T_last_r0[1,1]*np.cos(theta_1))#/(-np.sin(theta_5))
		sin = sgn_t_5*(T_last_r0[0,0]*np.sin(theta_1) - T_last_r0[1,0]*np.cos(theta_1))#/(-np.sin(theta_5))
		
		res = np.atan2(sin, cos)

		return res

	def theta_3_2_4(self, T_last_r0, theta_1, theta_5, theta_6, t3=None):
		rtn = []
		T_f1_r0 = self.T(1, theta_1)
		T_f5_r4 = self.T(5, theta_5)
		
		#print("INV T5-0:",T_last_r0)
		T_f4_r1 = np.matmul(self.inv_dh(T_f1_r0), T_last_r0)

		if(self.n_dof == 6):
			T_f6_r5 = self.T(6, theta_6)
			T_f4_r1 = np.matmul(T_f4_r1, self.inv_dh(T_f6_r5))

		T_f4_r1 = np.matmul(T_f4_r1, self.inv_dh(T_f5_r4))

		p4x = T_f4_r1[0,3]
		p4z = T_f4_r1[2,3]

		try:
			# theta 3
			p4xz_norm = np.sqrt(p4z**2+(p4x-self.a[2])**2)
			t_3 = np.acos(clamp((p4xz_norm**2 - self.a[3]**2 - self.a[4]**2)/(2*self.a[3]*self.a[4]),-1.0,1.0))

			t_3_list = [t_3, -t_3]
			
			if t3 is not None:
				if t3*t_3_list[0] >= 0:
					t_3_list.pop(1)
				else:
					t_3_list.pop(0)
			for theta_3 in t_3_list:
				try:
					# theta 2
					phi_3 = np.pi - theta_3
					phi_1 = np.atan2(p4z, (p4x-self.a[2]))
					phi_2 = np.asin(clamp(self.a[4]*np.sin(phi_3)/np.sqrt(p4z**2+(p4x-self.a[2])**2),-1.0,1.0))
					theta_2 = phi_1 - phi_2

					# theta_4
					T_f3_r2 = self.T(3, theta_3)
					T_f2_r1 = self.T(2, theta_2)

					#print("(",theta_1*180/np.pi,",",theta_2*180/np.pi,",",theta_3*180/np.pi,") INV T4-1:", T_f4_r1 )
					#print("INV T4-2:",np.matmul(self.inv_dh(T_f2_r1) , T_f4_r1 ))
					T_f1_r3 = np.matmul(self.inv_dh(T_f3_r2), self.inv_dh(T_f2_r1))
					T_f4_r3 = np.matmul(T_f1_r3, T_f4_r1)

					#print("INV T4-3:",T_f4_r3)#np.matmul(T_f4_r1, self.inv_dh(T_f2_r1)))
					
					theta_4 = -np.atan2(T_f4_r3[0,1], T_f4_r3[0,0])
					rtn.append([theta_2, theta_3, theta_4])
				except Exception as ex:
					pass
		except Exception as ex:
			pass
		return rtn

	def adjust_degree(self, d):
		d = d%360
		return d - 360* (d > 180)

	def adjust_radian(self, r):
		r = r%(2*np.pi)
		return r - (2*np.pi)*(r>np.pi)


"""
Euler ZYX mobile
alpha: around z
beta: around mobile y
gamma: around mobile x
"""

class Kinematic(Dof):
	"""docstring for Dorna_c_knmtc"""
	def __init__(self, model="dorna_2"):
		super(Kinematic, self).__init__()
		
		# create the 6 degree of freedom robot
		self.model = model

		if self.model=="dorna_2s":
			self.n_dof = 5 # number of degrees of freedom, choose between [5,6]
			self.alpha = [0, 0, np.pi/2, 0, 0, 0, 0] 
			self.delta = [0, 0, 0, 0, 0, np.pi/2, 0] 
			self.a = [0, 0 , 95.48, 203.2, 152.4, 0, 0]
			self.d = [0, 218.47, 0, 0, 0,48.92, 0]
			
		if self.model=="dorna_2":
			self.n_dof = 5 
			self.alpha = [0, 0, np.pi/2, 0, 0, 0, 0] 
			self.delta = [0, 0, 0, 0, 0, np.pi/2, 0] 
			self.a = [0, 0 , 95.48, 203.2, 152.4, 0, 0]
			self.d = [0, 206.4, 0, 0, 0,48.92, 0]

		if self.model=="dorna_ta":
			self.n_dof = 6
			self.alpha =  [0, np.pi/2, 0, np.pi/2, np.pi/2, np.pi/2, 0] 
			self.delta = [0, 0, 0, np.pi/2, np.pi, np.pi,0]
			self.a = [0.0, 0.08*1000, 0.21*1000, 0.*1000, 0., 0., 0.]
			self.d = [0.230 * 1000, 0., 0., 0.0418 * 1000, 0.17500 * 1000, -0.089 * 1000, 0.035 * 1000]


	def joint_to_theta(self, joint):
		theta = list(joint)
		return [np.radians(j) for j in theta]


	def theta_to_joint(self, theta):
		joint = [np.degrees(t) for t in theta]
		return [self.adjust_degree(j) for j in joint]

	def fw(self, joint):

		# adjust theta to dof
		_theta = self.joint_to_theta(joint)
		if self.n_dof == 5:
			_theta[5] = joint[5]

		# fw result
		fw = self.fw_base(_theta)
		
		#fw = np.matmul(self.T_rail_r_world , fw)

		self.set_matrix(fw)
		#abc = self.get_euler()
		abc = self.mat_to_axis_angle(fw)
		#abc = [np.degrees(r) for r in abc]

		#give different result: fw, fw can later be changed to pos + abg
		return [fw[0,3], fw[1,3], fw[2,3]] + abc


	def inv(self, xyzabc, joint_current=None, all_sol=False, freedom = None): #xyzabg
		#print("inv call:",xyzabc)
		ABC = xyzabc[3:].copy()
		
		if(ABC[0]==None or ABC[1]==None or ABC[2]==None):
			ABC =[0,0,0]

		rot = self.axis_angle_to_mat(ABC)

		T_tcp_r_world = np.matrix([
			[rot[0,0], rot[0,1], rot[0,2], xyzabc[0]],
			[rot[1,0], rot[1,1], rot[1,2], xyzabc[1]],
			[rot[2,0], rot[2,1], rot[2,2], xyzabc[2]],
			[0, 0, 0, 1]
		])

		# init condition
		theta_current = None
		if joint_current is not None:
			theta_current = list(joint_current)
			if theta_current:
				theta_5 = theta_current[5]
				theta_current = self.joint_to_theta(theta_current)
				if(self.n_dof==5):
					theta_current[5] = theta_5

		theta_all = self.inv_base(np.array(T_tcp_r_world), theta_current=theta_current, all_sol=all_sol , freedom = freedom)
		#theta_all = self.approach(np.array(T_tcp_r_world), theta_current=theta_current)


		# all the solution
		joint_all = np.array([self.theta_to_joint(theta) for theta in theta_all ])
		#if(self.n_dof==5):
		#	joint_all = [self.theta_to_joint(theta[:5]) + [theta[5]] for theta in theta_all ]

		return joint_all

def main_dorna_c():
	
	knmtc = Kinematic("dorna_ta")
	#knmtc.set_tcp_xyzabc([0, 0, 43, 0, 0, 90])

	joint = [60,60,0,0,0,60]

	fw = knmtc.fw(joint)

	print(knmtc.xyzquat_to_xyzabc([0.194881896*1000, 0.206358182*1000, 0.053252138000000004*1000, 0.01803234197700197, 0.9667316721684709, 0.25165134004776635, -0.04214631325916844] )
)
	#print(knmtc.xyzquat_to_xyzabc([0,0,0,1,0,0,0]))
	#ik_result = knmtc.inv(xyzabc, joint, False,freedom)


	#print("ik_result: ", ik_result)
	"""
	above_pick_pose = np.array([ 316.69730436, -308.54856831,  250.7750391,   180.,            0.,0.        ])
	above_place_pose = np.array([ 281.77910814,  -5.09010725,  330.87343997, -151.,           90.,-4.        ])
	quaternion1 = knmtc.mat_to_quat(knmtc.xyzabc_to_mat(above_pick_pose ))
	quaternion2 = knmtc.mat_to_quat(knmtc.xyzabc_to_mat(above_place_pose ))


	middle_quaternion = knmtc.quat_slerp(quaternion1, quaternion2, 0.5)
	middle_mat = np.eye(4)
	middle_mat[:3,:3] = knmtc.quat_to_mat(middle_quaternion)
	middle_xyzabc =  knmtc.mat_to_xyzabc(middle_mat)

	middle_xyzabc[:3] = (above_pick_pose[:3]  + above_place_pose[:3])/2

	print(middle_xyzabc)
	"""

if __name__ == '__main__':
	#main_random()
	#main_diagnose()
	main_dorna_c()

