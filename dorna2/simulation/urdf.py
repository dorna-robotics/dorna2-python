from urdfpy import URDF
from scipy.spatial.transform import Rotation as R
from node import Node
import numpy as np
import fcl

class UrdfRobot:

    def urdf_joint_transform(self, joint, value=0.0):
        """Get joint transform at a given joint value (for revolute or prismatic)."""
        T = np.eye(4)
        T = joint.origin

        axis = np.array(joint.axis)

        if joint.joint_type == 'revolute':
            R_axis = R.from_rotvec(value * axis).as_matrix()
            motion = np.eye(4)
            motion[:3, :3] = R_axis
            return T @ motion

        elif joint.joint_type == 'prismatic':
            motion = np.eye(4)
            motion[:3, 3] = value * axis
            return T @ motion

        else:  # fixed or unsupported
            return T

    def __init__(self, urdf_file, joint_values={}):
        self.robot = URDF.load(urdf_file)
        link_nodes = {}

        def recurse(link_name, parent_node):
            link = self.robot.link_map[link_name]
            joint = next((j for j in self.robot.joints if j.child == link_name), None)
            joint_value = joint_values.get(joint.name, 0.0) if joint else 0.0

            local_tf = self.urdf_joint_transform(joint, joint_value) if joint else np.eye(4)
            node = Node(link.name, parent_node, local_tf)
            node.lock = True

            link_nodes[link.name] = node

            # Visuals and Collisions
            for vis in link.visuals:
                node.visuals.append(vis.geometry)  # optionally convert to your own mesh format

            for col in link.collisions:
                shape = col.geometry
                tf = np.eye(4)            
                if col.origin is not None:
                    tf = np.matrix(col.origin)


                # Example: only support box/sphere/cylinder for now
                if shape.box:
                    half_extents = shape.box.size / 2
                    obj = (fcl.Box(*half_extents)) #fcl.CollisionObject
                elif shape.sphere:
                    obj = (fcl.Sphere(shape.sphere.radius))
                elif shape.cylinder:
                    obj = (fcl.Cylinder(shape.cylinder.radius, shape.cylinder.length))
                else:
                    continue
                node.collisions.append((obj,tf))

            for j in self.robot.joints:
                if j.parent == link_name:
                    recurse(j.child, node)

        self.root_link = self.robot.base_link
        self.root_node = Node(self.root_link.name)
        link_nodes[self.root_link.name] = self.root_node
        recurse(self.root_link.name, self.root_node)

    def set_joint_values(self, joints = [0,0,0,0,0,0,0]):

        def recurse(link_name, parent_node, joints, count):
            link = self.robot.link_map[link_name]
            joint = next((j for j in self.robot.joints if j.child == link_name), None)
            
            if count>-1:
                joint_value = joints[count] * np.pi / 180.0
            else:
                joint_value = 0

            local_tf = self.urdf_joint_transform(joint, joint_value) if joint else np.eye(4)
            node = parent_node.children[0]
            node.lock = True
            node.set_local_transform(local_tf)

            for j in self.robot.joints:
                if j.parent == link_name:
                    recurse(j.child, node, joints, count+1)

        recurse(self.root_link.name, self.root_node, joints, -1 )