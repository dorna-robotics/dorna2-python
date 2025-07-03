import numpy as np
import fcl

import trimesh
import pybullet as p


def load_mesh(filepath):
    # Load with trimesh
    mesh = trimesh.load(filepath, force='mesh')
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError("Loaded file does not contain a single mesh.")
    
    # Ensure triangulated faces
    mesh = mesh.as_triangles()
    
    # Vertices and triangles
    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.int32)
    
    return vertices, faces


#some good functions
def fcl_transform_from_matrix(matrix4x4):
    rotation = matrix4x4[:3, :3]
    translation = matrix4x4[:3, 3]
    return fcl.Transform(rotation, translation)


class Shape:
    def __init__(self, parent_node_, local_transform_):
        self.parent_node = parent_node_
        self.local_transform = local_transform_


class VisualShape(Shape):
    def __init__(self, parent_node_, local_transform_):
        super().__init__(parent_node_, local_transform_)


class CollisionShape(Shape):
    def __init__(self, parent_node_, local_transform_, fcl_collision_shape_, fcl_collision_object_):
        super().__init__(parent_node_, local_transform_)
        self.fcl_collision_shape = fcl_collision_shape_
        self.fcl_collision_object = fcl_collision_object_


class Node:

    def __init__(self, name, parent=None, local_transform=None, tags=[], target_tags=[]):
        self.name = name
        self.parent = None
        self.children = []
        self.lock = False
        self.local_transform = local_transform if local_transform is not None else np.eye(4)

        # Geometry
        self.visuals = []  # List of mesh or shape representations (could be your own format)
        self.collisions = []  # List of collision objects

        self.tags = set(tags)
        self.target_tags = set(target_tags)

        if parent:
            self.set_parent(parent)


    def get_global_transform(self):
        if self.parent:
            return self.parent.get_global_transform() @ self.local_transform
        return self.local_transform


    def set_local_transform(self, transform):
        self.local_transform = transform


    def set_parent(self, new_parent):
        if self.parent:
            self.parent.children.remove(self)
        self.parent = new_parent
        new_parent.children.append(self)


    def move_to(self, new_parent):
        """Reparents node while keeping global transform unchanged."""
        if self.lock: #locked nodes such as robot nodes, wont move around in tree
            print("Simulation Warning: Reparenting locked nodes is not possible!")
            return

        global_tf = self.get_global_transform()
        inv_new_parent_global = np.linalg.inv(new_parent.get_global_transform())
        new_local = inv_new_parent_global @ global_tf
        self.set_parent(new_parent)
        self.local_transform = new_local


    #add a new collision shape to this node, obj is a fcl collision obj. tf is the 4x4 matrix transformation in
    #the nodes local coordiante system
    def add_collision(self, obj, tf): 
        t1 = fcl_transform_from_matrix(self.get_global_transform() @ tf)
        collision_obj = fcl.CollisionObject(obj, t1)
        self.collisions.append(CollisionShape(self,tf, obj, collision_obj))


    def update_collision_object_transforms(self):
        gt = self.get_global_transform()
        for coll_shape in self.collisions:
            new_tf = fcl_transform_from_matrix(gt @ coll_shape.local_transform)
            coll_shape.fcl_collision_object.setTransform(new_tf)