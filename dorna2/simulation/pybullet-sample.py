import numpy as np 
import pybullet as p 
import pybullet_data 
import time 
from scipy.spatial.transform import Rotation as R 
from urdfpy import URDF 
import fcl
import time 
from pathGen import pathGen
from node import Node
from node import create_cube

#from simulation import Simulation

from dorna2.simulation import check_path

spawned_visuals = []

def draw_node_shapes(node): 
    tf = node.get_global_transform()
    pos = tf[:3, 3].tolist()
    orn_mat = tf[:3, :3]
    quat = R.from_matrix(orn_mat).as_quat().tolist()
    # draw collision shapes as transparent visuals
    for coll_shape in node.collisions:
        geom = coll_shape.fcl_collision_shape
        shape_tf = tf @ coll_shape.local_transform
        shape_pos = shape_tf[:3,3].flatten().tolist()[0]
        shape_quat = R.from_matrix(shape_tf[:3,:3]).as_quat().tolist()

        if isinstance(geom, fcl.Box):
            half = geom.side / 2.0
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half.tolist(), rgbaColor=[0,1,0,0.3])
        elif isinstance(geom, fcl.Sphere):
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=geom.radius, rgbaColor=[0,0,1,0.3])
        elif isinstance(geom, fcl.Cylinder):
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=geom.radius, length=geom.l, rgbaColor=[1,1,0,0.3])
        else:
            continue
        body_id = p.createMultiBody(baseVisualShapeIndex=vis, basePosition=shape_pos, baseOrientation=shape_quat)
        spawned_visuals.append(body_id)

    # recurse
    for child in node.children:
        draw_node_shapes(child)

def clear_shapes():
    for bid in spawned_visuals:
        p.removeBody(bid)
    spawned_visuals.clear()



# PyBullet setup 
p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-9.81)



# defininf simulation



"""
# motion 
sim.scene_clear()

box1 = CollisionShape()
box2 = CollisionShape()

sim.root_node.collisions = [box1,  box2]
sim.robot.nodes["tcp"] = [box3]

res = motion_gen.line_move(j1 , pose, tcp , step = 100)
"""

scene = [] 
for i in range(4):
    t = i * np.pi / 2 + np.pi/4
    l = 0.48
    tf = [l*np.cos(t),l*np.sin(t),0,0,0,0]

    scene.append(create_cube(tf, [0.05,0.05,0.8]))

tool = [create_cube([0,0,0,0,0,0], [0.02,0.02,0.3])]


sim, path, all_visuals, col_res = Simulation.check_path(motion="lmove", start_joint=[-90,0,0,0,0,0,0], end_joint=[0,0,0,0,0,0,0], scene=scene, tool=tool, tcp=[0,0,0,0,0,0], steps=30, early_exit=True)
print(col_res)
sim.preview_animation(path, all_visuals)
"""
start_time = time.time()

tool = load()
vial = cube(positon, scale)
tool = mesh("mesh.stl", pose, scale = [1,1,1], bb= False)
cyl = cyliner(pose, scale = [1,1,1])


res = Simulation.check_path(motion="lmove", start_joint=[-45,0,0,0,0,0,0], end_joint=[45,0,0,0,0,0,0], tcp=tcp, tool=[tool , vial], scene=[cylinder, cube])
res = Simulation.check_path(motion = "lmove", [-45,0,0,0,0,0,0], [45,0,0,0,0,0,0], tcp,  [tool ], environment)

print(res)

print("--- %s seconds ---" % (time.time() - start_time))

"""
