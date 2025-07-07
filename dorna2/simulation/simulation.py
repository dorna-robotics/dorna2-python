import numpy as np
import node
from urdf import UrdfRobot
import fcl
from pathGen import pathGen
from dorna2 import Dorna
import time 
import dorna2.pose as dp
import pybullet as p
#must be deleted:
import inspect


class Simulation:

	def __init__(self, name):
		self.root_node = node.Node("root")
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

			n = node.Node("column"+str(i), self.root_node, tf, [], [])

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


	def preview_animation(self, path, all_visuals):
		t = 0
		while True:
			j = path.path.get_point(t%1.0)

			self.robot.set_joint_values([j[6],j[0],j[1],j[2],j[3],j[4],j[5]]) 

			for a in all_visuals:
				prnt_mat = np.eye(4)

				if id(a.fcl_shape) in self.robot.prnt_map:
					prnt_mat = self.robot.prnt_map[id(a.fcl_shape)].get_global_transform()

				gmat = prnt_mat@a.mat
				xyzabc = dp.T_to_xyzabc(gmat)

				translation = [float(xyzabc[0]),float(xyzabc[1]),float(xyzabc[2])]  # fcl.Vector3f
				rotation = dp.rmat_to_quat(gmat[:3,:3])
				p.resetBasePositionAndOrientation(a.pybullet_id, translation, rotation)

			t = t + 0.01
			time.sleep(1./240.)


def check_path(motion, start_joint, end_joint, tcp=[0,0,0,0,0,0], tool=[], scene=[], steps=50, early_exit=False):
	sim = Simulation("tmp")

	all_visuals = [] #for visualization
	all_objects = [] #to create bvh
	dynamic_objects = [] #to update bvh

	#placing scene objects
	for obj in scene:
		sim.root_node.collisions.append(obj.fcl_object)
		all_objects.append(obj.fcl_object)
		all_visuals.append(obj)

	#placing tool objects
	for obj in tool:
		sim.robot.link_nodes["j6_link"].collisions.append(obj)
		sim.robot.all_objs.append(obj)
		sim.robot.prnt_map[id(obj.fcl_shape)] = sim.robot.link_nodes["j6_link"]

	#registering robot ojbects
	for obj in sim.robot.all_objs:
		dynamic_objects.append(obj)
		all_objects.append(obj.fcl_object)
		all_visuals.append(obj)
		
	manager = fcl.DynamicAABBTreeCollisionManager()
	manager.registerObjects(all_objects)  # list of all your CollisionObjects
	manager.setup()  # Builds the BVH tree

	path = pathGen(motion, start_joint, end_joint, steps, sim.dorna.kinematic, tcp)

	sample_points = steps
	start_time = time.perf_counter()

	col_res = []

	for i in range(sample_points):

		t = float(i) / float(sample_points - 1 ) 
		j = path.path.get_point(t%1.0)
		j6 = 0 #check for rail

		if len(j)>6:
			j6 = j[6]

		sim.robot.set_joint_values([j6,j[0],j[1],j[2],j[3],j[4],j[5]]) 

		#update dynamics
		for do in dynamic_objects:
			manager.update(do.fcl_object)

		cdata = fcl.CollisionData()

		def my_collect_all_callback(o1, o2, cdata):
			fcl.collide(o1, o2, cdata.request, cdata.result)
			return False

		manager.collide(cdata, my_collect_all_callback)#fcl.defaultCollisionCallback)

		tmp_res = None

		for contact in cdata.result.contacts:
		    # Extract collision geometries that are in contact
		    coll_geom_0 = contact.o1
		    coll_geom_1 = contact.o2

		    prnt0 = None
		    prnt1 = None

		    num_parents = 0

		    if id(coll_geom_0) in sim.robot.prnt_map:
		    	prnt0 = sim.robot.prnt_map[id(coll_geom_0)]
		    	num_parents = num_parents + 1

		    if id(coll_geom_1) in sim.robot.prnt_map:
		    	prnt1 = sim.robot.prnt_map[id(coll_geom_1)]
		    	num_parents = num_parents + 1


		    #this collision has nothing to do with robot
		    if num_parents == 0:
		    	continue 

		    #this is external collision, good to go
		    if num_parents == 1: 
		    	pass

		    #this is internal collision, needs to be filtered
		    if num_parents == 2:
		    	if prnt0.parent == prnt1 or prnt1.parent == prnt0 or prnt0 == prnt1:
		    		continue

		    #if here, meaning that a valid collision has been detected
		    tmp_res = ['scene' if prnt0 is None else prnt0.name, 'scene' if prnt1 is None else prnt1.name]
		    break

		if tmp_res is None:
			continue

		col_res.append({"t":t,"links":tmp_res,"joint":j})

		if early_exit:
			break

	ret = {"col":col_res, "singular":path.singular, "path":path.path.points,
			"debug":{"sim":sim, "path":path,"visuals":all_visuals}, "prf_time":(time.perf_counter() - start_time)*1000}

	return ret

def create_cube(pose, scale=[1,1,1]):
	return node.create_cube(pose, scale)

def create_sphere(pose, scale=[1,1,1]):
	return node.create_sphere(pose, scale)

def create_mesh(mesh_path, pose, scale=[1,1,1]):
	return node.create_mesh(mesh_path, pose, scale)