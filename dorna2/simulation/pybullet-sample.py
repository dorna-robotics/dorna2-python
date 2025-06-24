import numpy as np 
import pybullet as p 
import pybullet_data 
import time 
from scipy.spatial.transform import Rotation as R 
from urdfpy import URDF 
import fcl

from node import Node
from urdf import UrdfRobot

spawned_visuals = []

def draw_node_shapes(node): 
    tf = node.get_global_transform()
    pos = tf[:3, 3].tolist()
    orn_mat = tf[:3, :3]
    quat = R.from_matrix(orn_mat).as_quat().tolist()
    # draw collision shapes as transparent visuals
    for col, ltf in node.collisions:
        geom = col
        shape_tf = tf @ ltf
        shape_pos = shape_tf[:3,3].flatten().tolist()[0]
        shape_quat = R.from_matrix(shape_tf[:3,:3]).as_quat().tolist()

        if isinstance(geom, fcl.Box):
            half = geom.side #/ 2.0
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

urdf_path = 'C:/Users/jvd/Desktop/sphere-robot-collision/dextron-main/DornaTA/urdf/DornaTA_500mm_hd_rail.urdf'
# build custom tree
robot = UrdfRobot(urdf_path)
# initial draw
draw_node_shapes(robot.root_node)

j = 0
while True:
    # clear and redraw
    p.removeAllUserDebugItems()
    clear_shapes()
    draw_node_shapes(robot.root_node)

    j  = j + 0.01
    robot.set_joint_values([0,j,0,0,0,0,0]) 
    p.stepSimulation()
    time.sleep(1./240.)

