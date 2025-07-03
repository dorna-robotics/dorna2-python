import numpy as np
from node import Node
from urdf import UrdfRobot
import fcl
from pathGen import pathGen
from dorna2 import Dorna

class Simulation:

	def __init__(self, name):
		self.root_node = Node("root")
		self.init_robot()
		self.dorna = Dorna()



	def init_robot(self):
		urdf_path = 'C:/Users/jvd/Desktop/sphere-robot-collision/dextron-main/DornaTA/urdf/DornaTA_500mm_hd_rail.urdf'
		self.robot = UrdfRobot(urdf_path, {}, self.root_node)


	#checks if two nodes want to collide
	def wants_to_collide(self, c1, c2):
		#no node's collision with its parent need to be checked
		if c1.parent == c2 or c2.parent == c1:
			return False
		#no node's collision with itself should be checked
		if c1 is c2:
			return False
		# If either one has no target list, assume it allows everything
		if c1.target_tags is None and c2.target_tags is None:
			return True
		# nodes with empty target tags should be checked
		if len(c1.target_tags)==0 or len(c2.target_tags)==0:
			return True

		# If either one allows the other, it's allowed
		if (c1.target_tags is None or c2.tags & c1.target_tags) or (c2.target_tags is None or c1.tags & c2.target_tags):
			return True

		return False


	#checks collision between two nodes
	def check_nodes_collision(self, n1, n2):
		if not self.wants_to_collide(n1, n2):
			return False

		col = False
		for colshape1 in n1.collisions:
			for colshape2 in n2.collisions:
				req = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
				res = fcl.CollisionResult()
				collided = fcl.collide(colshape1.fcl_collision_object, colshape2.fcl_collision_object, req, res)
				if collided:
					return True
		return False


	#checking if robot has internal collision
	def robot_self_collision(self):
		links = self.robot.link_names

		for n1 in links:
			for n2 in links:
				
				if n1==n2:
					continue

				node1 = self.robot.link_nodes[n1]
				node2 = self.robot.link_nodes[n2]

				res = self.check_nodes_collision(node1,node2)

				if res:
					return True

		return False


	#checking if an external node has collision with robot nodes
	def node_robot_collision(self, node):
		links = self.robot.link_names
		
		for n in links:
				robot_node = self.robot.link_nodes[n]
				res = self.check_nodes_collision(node, robot_node)

				if res:
					return True

		return False


	def robot_external_collision(self):
		def recruse(node):

			res = self.node_robot_collision(node)

			if res:
				return [True, node]

			for c in node.children:

				if c==self.robot:
					continue

				child_res = recruse(c)

				if child_res[0]:
					return child_res

			return [False,None]

		return recruse(self.root_node)


	def create_simple_scene(self):
		for i in range(4):
			t = i * np.pi / 2
			l = 0.2
			tf = np.matrix([[1,0,0,l*np.cos(t)],[0,1,0,l*np.sin(t)],[0,0,1,0],[0,0,0,1]])

			n = Node("column"+str(i), self.root_node, tf, [], [])

			half_extents = [0.05,0.05,0.8]
			obj = fcl.Box(*half_extents)

			n.add_collision(obj, tf)


	def check_motion_collision(self, path):
		sample_points = 100
		collision_res = {"col":False,"col_time":0 , "internal":False, "external":False}
		for i in range(sample_points):

			t = float(i) / float(sample_points - 1 ) 
		    
			j = path.path.get_point(t%1.0)

			j6 = 0 #check for rail
			if len(j)>6:
				j6 = j[6]

			self.robot.set_joint_values([j6,j[0],j[1],j[2],j[3],j[4],j[5]]) 

			external_col = self.robot_external_collision() 
			internal_col = self.robot_self_collision()

			if external_col[0] or internal_col:
				collision_res["col"] = True
				collision_res["col_time"] = t
				collision_res["internal"] = internal_col
				collision_res["external"] = external_col

				break

		return collision_res


	def check_path(self, motion, j1, j2, steps = 100):
		path = pathGen(motion, j1, j2, steps, self.dorna.kinematic)

		res = {}

		res["path"] = path.path
		res["points"] = path.path.points
		res["singular"] = path.singular
		res["collision"] = self.check_motion_collision(path)

		return res

