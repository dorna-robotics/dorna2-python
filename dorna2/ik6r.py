import numpy as np
import math
from dorna2.cf import CF
from sympy.parsing.mathematica import mathematica
from sympy import var
from dorna2.ik6r_matrix import sigma_matrix
from poly import poly
import time


def matrix_multiply(A, B):#only works for our case
    # Initialize result matrix with zeros
	result = [[poly([0]) for _ in range(len(B[0]))] for _ in range(len(A))]

	# Perform matrix multiplication
	for i in range(len(A)):
		for j in range(len(B[0])):
			for k in range(i,len(B)):
				result[i][j] += A[i][k] * B[k][j]

	return result

def mu_matrix(A):
	result = [[poly([0]) for _ in range(len(A))] for _ in range(len(A))]
	for i in range(len(A)):
		for j in range(i+1,len(A)):
				result[i][j] = A[i][j]

	for k in range(1, len(A)):
		m = len(A)-1-k
		result[m][m] = result[m+1][m+1] + A[m+1][m+1]*(-1.0)

	return result

def FA(x,A):
	return matrix_multiply(mu_matrix(x),A)

def copy_matrix(s):
	result = [[s[j][i].copy() for i in range(len(s))] for j in range(len(s))]
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

def ik(a2,a3,d1,d4,d5,d6,d7,mat):

	f11=mat[0][0]
	f12=mat[0][1]
	f13=mat[0][2]
	f14=mat[0][3]

	f21=mat[1][0]
	f22=mat[1][1]
	f23=mat[1][2]
	f24=mat[1][3]

	f31=mat[2][0]
	f32=mat[2][1]
	f33=mat[2][2]
	f34=mat[2][3]


	g = sigma_matrix(a2,a3,d1,d4,d5,d6,d7,f13,f23,f33,f14,f24,f34)
	result = copy_matrix(g)
	original_g = copy_matrix(g)
	
	for i in range(11):
		result=FA(result, g)

	det = result[0][0]
	det.normalize()
	print("det:",det)
	q,r = det.polydiv(det.coefficients, (poly([1,0,1])*poly([1,0,1])*poly([1,0,1])*poly([1,0,1])).coefficients)
	det = poly(q)
	print("r:",r)
	#det.normalize()
	print("q:",det)
	#small_poly = poly(det.coefficients[:16])
	#small_result = small_poly.coefficients

	roots = np.roots(det.coefficients[::-1])



	x3_list = roots.real[abs(roots.imag)<1e-5]

	res = []
	print("roots : ", roots)

	for x3 in x3_list:
		print("root:", x3, "has value: ", det.evaluate(x3))
		theta3 = 2*math.atan(x3)
		matrix = matrix_evaluated(original_g,x3)
		matrix_t = [row[:11] for row in matrix[:11]]
		inv_matrix_t = np.linalg.inv(np.array(matrix_t))
		matrix_right = np.array([[row[11] for row in matrix[:11]]]).T

		ff = np.matmul(inv_matrix_t, -matrix_right)
		print("ff:",ff)
		x5 = ff[10]
		x4 = ff[8]
		theta4 = 2*math.atan(x4)
		theta5 = 2*math.atan(x5)
		c3 = math.cos(theta3)
		s3 = math.sin(theta3)
		c4 = math.cos(theta4)
		s4 = math.sin(theta4)
		c5 = math.cos(theta5)
		s5 = math.sin(theta5)


		#print((f14*f23 - f13*f24))
		for i in range(2):
			c1 = (2*i-1)*(d4*f13 - d6*f13*c4 - d7*f13*s4*s5 + f14*s4*s5)#/(f14*f23 - f13*f24))
			s1 = (2*i-1)*((-d4)*f23 + d6*f23*c4 + d7*f23*s4*s5 - f24*s4*s5)#/((-f14)*f23 + f13*f24))
			theta1 = math.atan2(s1,c1)
			c1 = math.cos(theta1)
			s1 = math.sin(theta1)
			
			c2 =  -(((-f33)*(c5*s3 + c3*c4*s5) - (f13*c1 + f23*s1)*(c3*c5 -c4*s3*s5))/(f33**2 + f13**2*c1**2 + f23**2*s1**2 + f13*f23*math.sin(2*theta1)))
			s2 = ((-s3)*(f13*c1*c5 + f23*c5*s1 + f33*c4*s5) + c3*(f33*c5 - c4*(f13*c1 + f23*s1)*s5))/(f33**2 + f13**2*c1**2 + f23**2*s1**2 + f13*f23*math.sin(2*theta1))
			theta2 = math.atan2(s2,c2)

			c6 = (s1*(f12*c4 + f11*s5*s4) - c1*(f22*c4 + f21*c5*s4))/((f21**2 + f22**2)*c1**2 + s1*(-2*(f11*f21 + f12*f22)*c1 + (f11**2 + f12**2)*s1))
			s6 =  (s1*(f11*c4 - f12*c5*s4) + c1*((-f21)*c4 + f22*c5*s4))/((f21**2 + f22**2)*c1**2 + s1*(-2*(f11*f21 + f12*f22)*c1 + (f11**2 + f12**2)*s1))
			theta6 = math.atan2(s6,c6)

			res.append(np.array([theta1,theta2,theta3,theta4,theta5,theta6]))
	return res
	
	#print(np.array(matrix_1) - np.array(matrix_2))
	#print(determinant(g))

if __name__ == '__main__':

	start_time = time.time()
					#a2,a3,d1,d4,d5,d6,d7,f13,f23,f33,f14,f24,f34

	mat = [[-(1+np.sqrt(3))/(2*np.sqrt(2)),0,0.25*(np.sqrt(2)-np.sqrt(6)),0.25*(8+np.sqrt(2)*(3-np.sqrt(3)))]
			,[0,-1,0,0],
			[0.25*(np.sqrt(2)-np.sqrt(6)),0,0.25*(np.sqrt(2)+np.sqrt(6)),0.25*(4+np.sqrt(2)*(3+np.sqrt(3)))],
			[0,0,0,1]]
	start_time = time.time()
	ik(1,1,1,1,1,1,1,mat)
	elapsed_time = time.time() - start_time
	print(f"Elapsed time: {elapsed_time} seconds")
