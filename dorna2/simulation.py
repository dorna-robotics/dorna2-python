import numpy as np
import time 
import sys
import fcl

from dorna2 import Dorna
from dorna2.pathGen import pathGen
from dorna2.urdf import UrdfRobot
import dorna2.node as node
import dorna2.pose as dp
import pybullet as p



def pybullet_test():
	if 'p' in globals() and 'pybullet' in sys.modules and globals()['p'] is sys.modules['pybullet']:
		return True
	return False

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
		if not pybullet_test():
			return

		t = 0
		while True:
			j = path.get_point(t%1.0)

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


def check_path(motion, start_joint, end_joint, tool=[0,0,0,0,0,0], load=[], scene=[], steps=50, early_exit=False,
 base_in_world=[0,0,0,0,0,0], frame_in_world=[0,0,0,0,0,0], aux_dir=[[0, 0, 0], [0, 0, 0]]):
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
	for obj in load:
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

	path = pathGen(motion, start_joint, end_joint, steps, sim.dorna.kinematic, tool)

	sample_points = steps
	start_time = time.perf_counter()

	col_res = []

	base_in_world_mat = sim.dorna.kinematic.xyzabc_to_mat(base_in_world)
	frame_in_world_inv = sim.dorna.kinematic.inv_dh(sim.dorna.kinematic.xyzabc_to_mat(frame_in_world))

	aux_dir_1 = base_in_world_mat @ np.array([aux_dir[0][0], aux_dir[0][1], aux_dir[0][2],0])
	aux_dir_2 = base_in_world_mat @ np.array([aux_dir[1][0], aux_dir[1][1], aux_dir[1][2],0])

	for i in range(sample_points):

		t = float(i) / float(sample_points - 1 ) 
		j = path.path.get_point(t%1.0)
		if t == 1.0:
			j = path.path.get_point(1.0)


		base_mat = np.array(base_in_world_mat)
		aux_offset = np.array([0,0,0,0])


		if len(j)>6:
			aux_offset = aux_offset + j[6] * aux_dir_1
		if len(j)>7:
			aux_offset = aux_offset + j[7] * aux_dir_2

		base_mat[0, 3] += aux_offset[0, 0]
		base_mat[1, 3] += aux_offset[0, 1]
		base_mat[2, 3] += aux_offset[0, 2]

		sim.robot.set_joint_values([0,j[0],j[1],j[2],j[3],j[4],j[5]], frame_in_world_inv @  base_mat) 

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

def check_collision(joint, tool=[0,0,0,0,0,0], load=[], scene=[],
 base_in_world=[0,0,0,0,0,0], frame_in_world=[0,0,0,0,0,0], aux_dir=[[0, 0, 0], [0, 0, 0]]):
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
	for obj in load:
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


	col_res = []

	base_in_world_mat = sim.dorna.kinematic.xyzabc_to_mat(base_in_world)
	frame_in_world_inv = sim.dorna.kinematic.inv_dh(sim.dorna.kinematic.xyzabc_to_mat(frame_in_world))

	aux_dir_1 = base_in_world_mat @ np.array([aux_dir[0][0], aux_dir[0][1], aux_dir[0][2],0])
	aux_dir_2 = base_in_world_mat @ np.array([aux_dir[1][0], aux_dir[1][1], aux_dir[1][2],0])

	j = joint


	base_mat = np.array(base_in_world_mat)
	aux_offset = np.array([0,0,0,0])


	if len(j)>6:
		aux_offset = aux_offset + j[6] * aux_dir_1
	if len(j)>7:
		aux_offset = aux_offset + j[7] * aux_dir_2

	base_mat[0, 3] += aux_offset[0, 0]
	base_mat[1, 3] += aux_offset[0, 1]
	base_mat[2, 3] += aux_offset[0, 2]

	sim.robot.set_joint_values([0,j[0],j[1],j[2],j[3],j[4],j[5]], frame_in_world_inv @  base_mat) 

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

	if tmp_res is not None:
		col_res.append({"links":tmp_res})


	ret = {"col":col_res}

	return ret

def create_cube(pose, scale=[1,1,1]):
	return node.create_cube(pose, scale)

def create_sphere(pose, scale=[1,1,1]):
	return node.create_sphere(pose, scale)

def create_mesh(mesh_path, pose, scale=[1,1,1]):
	return node.create_mesh(mesh_path, pose, scale)


def dedupe_events(events, tol=1e-4):
	"""
	Deduplicate a list of event dicts (with keys 't' and 'd').
	- Sorts by t and distance.
	- Merges events whose t differ by <= tol.
	- Keeps the event with the smallest distance in each group.

	Args:
		events (list of dict): each dict must have 't' (float) and 'd' (float).
		tol (float): tolerance on t for merging (default 1e-4).

	Returns:
		list of dict: deduplicated events.
	"""
	if not events:
		return []

	# sort first by t, then by distance
	events = sorted(events, key=lambda e: (e['t'], e['d']))

	deduped = []
	i = 0
	while i < len(events):
		best = events[i]
		j = i + 1
		while j < len(events) and abs(events[j]['t'] - events[i]['t']) <= tol:
			if events[j]['d'] < best['d']:
				best = events[j]
			j += 1
		deduped.append(best)
		i = j

	return deduped

def check_path_bisect_from_path(
	path,								   # has .get_point(t) in [0,1]
	tool=[0,0,0,0,0,0],
	load=[],
	scene=[],
	dist_thresh=0.01,					   # meters; threshold for refinement
	max_depth=3,							 # max recursive bisection depth
	base_in_world=[0,0,0,0,0,0],
	frame_in_world=[0,0,0,0,0,0],
	aux_dir=[[0,0,0],[0,0,0]],			   # two optional aux axes in world (same as your check_path)
	keep_near_count=3,					   # keep a few best minima per segment for debugging
):
	"""
	Returns:
	  {
		'events': [ {'t':t, 'q':q, 'dmin':d, 'pair':(nameA,nameB), 'pA':pA, 'pB':pB, 'n':n} ... ],
		'mins':   [ same as above, local minima sampled during refinement ],
		'debug':  {'sim': sim, 'visuals': all_visuals},
		'prf_time': ms
	  }
	"""
	start_time = time.perf_counter()

	# ---- 0) Build a one-off Simulation & BVH identical to your style ----
	sim = Simulation("tmp")

	all_visuals = [] #for visualization
	all_objects = [] #to create bvh
	all_obj = []
	dynamic_objects = [] #to update bvh

	#placing scene objects
	for obj in scene:
		sim.root_node.collisions.append(obj.fcl_object)
		all_objects.append(obj.fcl_object)
		all_obj.append(obj)
		all_visuals.append(obj)

	#placing tool objects
	for obj in load:
		sim.robot.link_nodes["j6_link"].collisions.append(obj)
		sim.robot.all_objs.append(obj)
		sim.robot.prnt_map[id(obj.fcl_shape)] = sim.robot.link_nodes["j6_link"]

	#registering robot ojbects
	for obj in sim.robot.all_objs:
		dynamic_objects.append(obj)
		all_objects.append(obj.fcl_object)
		all_obj.append(obj)
		all_visuals.append(obj)

	manager = fcl.DynamicAABBTreeCollisionManager()
	manager.registerObjects(all_objects)
	manager.setup()

	# transforms
	kin = sim.dorna.kinematic
	base_in_world_mat = kin.xyzabc_to_mat(base_in_world)
	frame_in_world_inv = kin.inv_dh(kin.xyzabc_to_mat(frame_in_world))

	aux_dir_1 = base_in_world_mat @ np.array([aux_dir[0][0], aux_dir[0][1], aux_dir[0][2], 0.0])
	aux_dir_2 = base_in_world_mat @ np.array([aux_dir[1][0], aux_dir[1][1], aux_dir[1][2], 0.0])

	# ---- 1) Pose setter for a given t in [0,1] ----
	def _set_pose(t):
		j = path.get_point(min(max(t, 0.0), 1.0))  # joint vector (degrees), possibly with aux components
		base_mat = np.array(base_in_world_mat)
		aux_offset = np.array([0,0,0,0.0])

		if len(j) > 6:
			aux_offset = aux_offset + j[6] * aux_dir_1
		if len(j) > 7:
			aux_offset = aux_offset + j[7] * aux_dir_2

		base_mat[0, 3] += aux_offset[0]
		base_mat[1, 3] += aux_offset[1]
		base_mat[2, 3] += aux_offset[2]

		# set robot pose (expects degrees for joints 1..6)
		sim.robot.set_joint_values(
			[0, j[0], j[1], j[2], j[3], j[4], j[5]],
			frame_in_world_inv @ base_mat
		)
		for do in dynamic_objects:
			manager.update(do.fcl_object)
		return np.array(j, dtype=float)

	# ---- 2) Distance oracle for current pose (worst pair & nearest points) ----
	dreq = fcl.DistanceRequest(enable_nearest_points=True)

	def _pair_ok(o1, o2):

		prnt0 = sim.robot.prnt_map.get(id(o1.fcl_shape), None)
		prnt1 = sim.robot.prnt_map.get(id(o2.fcl_shape), None)

		if prnt0 is None and prnt1 is None:
			return False
		if (prnt0 is not None) and (prnt1 is not None):
			if prnt0.parent == prnt1 or prnt1.parent == prnt0 or prnt0 == prnt1:
				return False
		return True

	# iterate explicit pairs (reliable across py-fcl builds)
	all_pairs = [(o1, o2) for (idx, o1) in enumerate(all_obj) for o2 in all_obj[idx+1:]]

	def _worst_distance_now():
		best = {'d': np.inf, 'pair': None, 'pA': None, 'pB': None, 'n': None}
		for (o1, o2) in all_pairs:
			if not _pair_ok(o1, o2):
				continue
			dres = fcl.DistanceResult()
			fcl.distance(o1.fcl_object, o2.fcl_object, dreq, dres)
			d = float(dres.min_distance)
			if d < best['d']:
				pA = np.asarray(dres.nearest_points[0], float)[:3]
				pB = np.asarray(dres.nearest_points[1], float)[:3]
				n = pB - pA
				ln = np.linalg.norm(n)
				n = n/ln if ln > 1e-12 else np.zeros(3)
				A = sim.robot.prnt_map.get(id(o1.fcl_shape), None)
				B = sim.robot.prnt_map.get(id(o2.fcl_shape), None)
				best.update({
					'd': d,
					'pair': (A.name if A else 'scene', B.name if B else 'scene'),
					'pA': pA, 'pB': pB, 'n': n
				})
		return best

	# convenience
	def eval_at_t(t):
		q = _set_pose(t)
		info = _worst_distance_now()  # {'d', 'pair', 'pA','pB','n'}
		return q, info

	# ---- 3) Initial uniform sampling: at least 3 bisections → 9 points ----
	t_samples = [i/8.0 for i in range(9)]  # 0,1/8,...,1
	samples = []
	for t in t_samples:
		q, info = eval_at_t(t)
		samples.append({'t': t, 'q': q, **info})

	# ---- 4) Recursive bisection over “hot” segments ----
	events = []   # where d <= dist_thresh
	mins = []	 # local minima seen

	def _bisect(t0, s0, t1, s1, depth):
		# s* are dicts like {'t','q','d','pair','pA','pB','n'}
		if depth >= max_depth:
			# At leaf: record the better endpoint if critical
			best = s0 if s0['d'] <= s1['d'] else s1
			if best['d'] <= dist_thresh:
				events.append(best)
			return

		tm = 0.5*(t0 + t1)
		q_m, info_m = eval_at_t(tm)
		sm = {'t': tm, 'q': q_m, **info_m}

		# track local min on this triple
		best = min([s0, sm, s1], key=lambda s: s['d'])
		mins.append(best)

		# refine if near/under threshold or a clear dip at mid
		refine = (s0['d'] <= dist_thresh or s1['d'] <= dist_thresh or sm['d'] <= dist_thresh
				  or sm['d'] < min(s0['d'], s1['d']) - 1e-6)

		if not refine:
			return
		_bisect(t0, s0, tm, sm, depth+1)
		_bisect(tm, sm, t1, s1, depth+1)

	# walk adjacent pairs of initial samples
	for a, b in zip(samples[:-1], samples[1:]):
		_bisect(a['t'], a, b['t'], b, depth=0)

	# ---- 5) Deduplicate & thin events (optional tidy-up) ----
	# sort by t and distance
	#events.sort(key=lambda e: (round(e['t'], 4), e['d']))

	deduped = []
	i = 0
	while i < len(events):
		best = events[i]
		j = i + 1
		while j < len(events) and round(events[j]['t'], 4) == round(events[i]['t'], 4):
			if events[j]['d'] < best['d']:
				best = events[j]
			j += 1
		deduped.append(best)
		i = j

	events = deduped
	# optionally keep only a few best minima per coarse segment (visualization-friendly)
	mins.sort(key=lambda e: (e['t'], e['d']))
	# (you can keep them all; trimming is optional)

	return {
		'events': events,
		'mins': mins[:keep_near_count*len(t_samples)],  # light trim
		'debug': {'sim': sim, 'visuals': all_visuals},
		'prf_time': (time.perf_counter() - start_time) * 1000.0
	}
