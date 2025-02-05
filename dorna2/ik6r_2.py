import numpy as np
import math
from dorna2.cf import CF
from dorna2.ik6r_matrix import sigma_matrix
from dorna2.poly import poly

#File containing for a 6-r manipulator with non-zero DH parameters: a2,a3,d1,-d4,d5,d6,d7
#The algorithm is originaly based on: Raghavan(1993): "Inverse Kinematics of the General 6R Manipulator and Related Linkages"
#The polynomial computation also takes inspiration from Gomez et al. "A faster algorithm for calculating the inverse kinematics of a general 6R manipulator for robot real time control"
#The calculation of determinant algorithm is based on: Bird(2011) "A simple division-free algorithm for computing determinants"


def copy_matrix(s):
	result = [[s[j][i] for i in range(len(s))] for j in range(len(s))]
	return result

def matrix_evaluated(A,x3):
	result = [[0 for i in range(12)] for j in range(12)]
	for i in range(12):
		for j in range(12):
			result[i][j] = A[i][j].evaluate(x3)
	return result


def pstr(matrix):
	for i in range(12):
		print(str(matrix[i][0]) + "," + str(matrix[i][1]) + "," + str(matrix[i][2]) + "," + str(matrix[i][3]) + "," + str(matrix[i][4]) + "," + str(matrix[i][5]) + "," + str(matrix[i][6]) + "," + str(matrix[i][7]) + "," + str(matrix[i][8]) + "," + str(matrix[i][9]) + "," + str(matrix[i][10]) + "," + str(matrix[i][11]))

def solve_cs_equation( aa,  bb,  cc,  i):
	#solving equation aa+ bb * cos(theta) + cc * sin(theta) = 0 for theta
	
	c1 = 0
	s1 = 0

	delta = cc * cc * (-aa * aa + bb * bb + cc * cc)
	if (delta < 0):
		return [False, 0,0]
	
	if (bb == 0.0 and cc == 0.0):
		return [False,0,0]

	if (bb == 0.0):
		s1 = -aa / cc
		if (abs(s1) > 1.0):
			return [False,0,0]

		c1 = np.sqrt(1.0 - s1 * s1)
		if (i == 1) :
			c1 = -c1
		
		return [True,c1,s1]
	
	if (cc == 0.0):
		c1 = -aa / bb
		if (abs(c1) > 1.0):
			return [False,0,0]
		s1 = sqrt(1.0 - c1 * c1)

		if (i == 1):
			s1 = -s1

		return [True, c1,s1]
	

	if (i == 0):
		c1 = (- aa * bb + np.sqrt(delta)) / (bb * bb + cc * cc)
	
	else:
		c1 = (-aa * bb -  np.sqrt(delta)) / (bb * bb + cc * cc)
	
	
	s1 = - (aa + bb * c1) / cc

	return [True,c1,s1]


def ik(a2,a3,d1,d4,d5,d6,d7,mat):

	f11 = mat[0,0]
	f12 = mat[0,1]
	f13 = mat[0,2]
	f14 = mat[0,3] / 100.0
	f21 = mat[1,0]
	f22 = mat[1,1]
	f23 = mat[1,2]
	f24 = mat[1,3] / 100.0
	f31 = mat[2,0]
	f32 = mat[2,1]
	f33 = mat[2,2]
	f34 = mat[2,3] / 100.0

	a2 = a2	/100.0 
	a3 = a3	/100.0 
	d1 = d1 /100.0 
	d4 = d4 /100.0 
	d5 = d5	/100.0 
	d6 = d6	/100.0 
	d7 = d7	/100.0 

	mat  = sigma_matrix(a2,a3,d1,d4,d5,d6,d7,f13,f23,f33,f14,f24,f34)

	A = np.zeros((12, 12))
	B = np.zeros((12, 12))
	C = np.zeros((12, 12))
	M = np.zeros((24, 24))
	for i in range(12):
		for j in range(12):
			A[i][j] = mat[i][j][2]
			B[i][j] = mat[i][j][1]
			C[i][j] = mat[i][j][0]
	

	A_inv = np.linalg.inv(A)

	B = np.matmul(A_inv, B)
	C = np.matmul(A_inv, C)

	for i in range(24):
		for j in range(24):
			if i < 12:
				if j - 12 == i :
					M[i][j] = 1
				
				else :
					M[i][j] = 0
				
				continue
			

			if (j < 12):
				M[i][j] = -C[i-12][j]
			
			else :
				M[i][j] = -B[i-12][j-12]
	

	eigenvalues, eigenvectors = np.linalg.eig(M)

	res = []

	for i, eigenvalue in enumerate(eigenvalues):
		if abs(eigenvalue.imag) > 0.9:
			continue

		x3 = eigenvalue.real
		theta3 = 2 * np.arctan(x3)		



		f5abs = abs(eigenvectors[1,i])
		f4abs = abs(eigenvectors[3,i])

		kval5 = 0
		kval4 = 0

		if (f5abs > 1e-4) :
			kval5 = eigenvectors[0, i] / eigenvectors[1, i]
		else:
			kval5 =eigenvectors[10,i] / eigenvectors[11, i]
		

		if (f4abs > 1e-4):
			kval4 = eigenvectors[ 0, i] /  eigenvectors[3, i]
		else :
			kval4 = eigenvectors[ 8, i] / eigenvectors[ 11, i]
		
	
	
		x5 = (kval5).real
		x4 = (kval4).real

		#print("- j2: ", theta3*180/np.pi , " ix4: ",(kval4).imag, " ix5: ",(kval5).imag )


		theta4 = 2.0 * np.arctan(x4)
		theta5 = 2.0 * np.arctan(x5)

		c1 = 0
		s1 = 0
		c2 = 0
		s2 = 0
		s6 = 0
		c6 = 0

		c3 = np.cos(theta3)
		s3 = np.sin(theta3)
		c4 = np.cos(theta4)
		s4 = np.sin(theta4)
		c5 = np.cos(theta5)
		s5 = np.sin(theta5)

		den = (f14 * f23 - f13 * f24)
		#print("\t\t den: ",den)
		for j in range(2):
			if abs(den) > 1e-5 :
				if j == 1:
					break
				
				c1 = ((-d4) * f13 + d6 * f13 * c4 + (d7 * f13 - f14) * s4 * s5) / den
				s1 = ((-d4) * f23 + d6 * f23 * c4 + (d7 * f23 - f24) * s4 * s5) / den
			
			else:
				mres = solve_cs_equation(-d4 + d6 * c4 , -(d7 * f23 - f24), -(-d7 * f13 + f14), j)
				c1 = mres[1]
				s1 = mres[2]

				if (not mres[0]):
					break
			

			theta1 = np.arctan2(s1, c1)
			c1 = np.cos(theta1)
			s1 = np.sin(theta1)

			den = (f33 * f33 + f13 * f13 * c1 * c1 + f23 * f23 * s1 * s1 + f13 * f23 * np.sin(2.0 * theta1))
			c2 = -(((-f33) * (c5 * s3 + c3 * c4 * s5) - (f13 * c1 + f23 * s1) * (c3 * c5 - c4 * s3 * s5)) / den)
			s2 = ((-s3) * (f13 * c1 * c5 + f23 * c5 * s1 + f33 * c4 * s5) + c3 * (f33 * c5 - c4 * (f13 * c1 + f23 * s1) * s5)) / den
			theta2 = np.arctan2(s2, c2)


			den = (c1 * c1 * (f21 * f21 + f22 * f22) - 2 * c1 * (f11 * f21 + f12 * f22) * s1 + (f11 * f11 + f12 * f12) * s1 * s1)
			c6 = (s1 * (c4 * f12 + c5 * f11 * s4) - c1 * (c4 * f22 + c5 * f21 * s4)) / den
			s6 = ((-c1) * c4 * f21 + c4 * f11 * s1 + c1 * c5 * f22 * s4 - c5 * f12 * s1 * s4) / den
			theta6 = np.arctan2(s6, c6)

			f_res = [theta1, theta2, theta3, theta4, theta5, theta6]
			res.append(f_res)
	return res
"""
			gsl_matrix* test_fw = gsl_matrix_alloc(4, 4);
			flange_r_base(test_fw, final_res[*num_res],false);

			double norm = 0.0;
			for (int m = 0; m < 4; m++) {
				for (int n = 0; n < 4; n++) {
					norm += pow(gsl_matrix_get(test_fw, m, n) - trans[m][n], 2.0);
				}
			}
			if (norm < 0.001) {
				(*num_res)++;
			}
"""



if __name__ == '__main__':

	mat = [[-(1+np.sqrt(3))/(2*np.sqrt(2)),0,0.25*(np.sqrt(2)-np.sqrt(6)),0.25*(8+np.sqrt(2)*(3-np.sqrt(3)))]
			,[0,-1,0,0],
			[0.25*(np.sqrt(2)-np.sqrt(6)),0,0.25*(np.sqrt(2)+np.sqrt(6)),0.25*(4+np.sqrt(2)*(3+np.sqrt(3)))],
			[0,0,0,1]]
