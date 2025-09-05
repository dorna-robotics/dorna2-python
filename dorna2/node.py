import numpy as np
import fcl
import sys
import trimesh
import pybullet as p
import dorna2.pose as dp

def pybullet_test():
    if 'p' in globals() and 'pybullet' in sys.modules and globals()['p'] is sys.modules['pybullet']:
        return True
    return False

def axis_angle_to_quaternion(axis_angle):
    theta = np.linalg.norm(axis_angle)
    if theta < 1e-6:
        return [0, 0, 0, 1]
    axis = axis_angle / theta
    x, y, z = axis
    theta = theta * 180.0 / np.pi
    s = np.sin(theta / 2.0)
    return [x * s, y * s, z * s, np.cos(theta / 2.0)]


def transform_to_matrix(xyz, rvec):
    quat = axis_angle_to_quaternion(rvec)
    
    # compatibility across FCL builds
    if hasattr(fcl, "Transform"):
        tf = fcl.Transform(quat, xyz)
    elif hasattr(fcl, "Transform3f"):
        tf = fcl.Transform3f(quat, xyz)
    elif hasattr(fcl, "Transformd"):
        tf = fcl.Transformd(quat, xyz)
    else:
        raise ImportError("No suitable Transform class found in fcl module.")
    
    return tf, quat


class unified_object:
    def __init__(self, fcl_obj, pybullet_id, mat, fcl_shape):
        self.fcl_object = fcl_obj
        self.pybullet_id = pybullet_id
        self.mat = mat
        self.fcl_shape = fcl_shape

#some good functions

def fcl_transform_from_matrix(matrix4x4):
    rotation = matrix4x4[:3, :3]
    translation = matrix4x4[:3, 3]

    # ensure numpy vector is proper type
    translation = np.array(translation, dtype=np.float32)

    # try to use rotation matrix directly if supported
    if hasattr(fcl, "Transform"):
        return fcl.Transform(rotation, translation)
    elif hasattr(fcl, "Transform3f"):
        try:
            return fcl.Transform3f(rotation, translation)
        except TypeError:
            # some builds want quaternion instead of rotation matrix
            from scipy.spatial.transform import Rotation as R
            quat = R.from_matrix(rotation).as_quat()  # [x, y, z, w]
            return fcl.Transform3f(quat, translation)
    elif hasattr(fcl, "Transformd"):
        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(rotation).as_quat()
        return fcl.Transformd(quat, translation)
    else:
        raise ImportError("No suitable Transform class found in fcl module.")

def create_cube(xyz_rvec, scale=[1,1,1]):
    xyz = xyz_rvec[:3]
    rvec = xyz_rvec[3:]
    tf, quat = transform_to_matrix(xyz,rvec)
    mat = dp.xyzabc_to_T(xyz_rvec)

    # FCL
    half_extents = [s / 2.0 for s in scale]
    box = fcl.Box(*scale)
    fcl_obj = fcl.CollisionObject(box, tf)

    # PyBullet
    body_id = 0
    if pybullet_test():
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[1,0,0,1])
        body_id = p.createMultiBody(baseVisualShapeIndex=vis_id, basePosition=xyz, baseOrientation=quat)

    return unified_object(fcl_obj, body_id, mat, box)


def create_mesh(mesh_path, xyz_rvec, scale=[1,1,1]):
    xyz = xyz_rvec[:3]
    rvec = xyz_rvec[3:]
    tf, quat = transform_to_matrix(xyz, rvec)
    mat = dp.xyzabc_to_T(xyz_rvec)

    # Load mesh
    mesh = trimesh.load(mesh_path, force='mesh').as_triangles()
    mesh.apply_scale(scale)
    
    vertices = mesh.vertices.tolist()
    triangles = [list(face) for face in mesh.faces]

    # FCL
    bvh = fcl.BVHModel()
    bvh.beginModel(len(vertices), len(triangles))
    bvh.addSubModel(vertices, triangles)
    bvh.endModel()
    fcl_obj = fcl.CollisionObject(bvh, tf)

    # Save mesh as temp .obj for PyBullet
    with tempfile.NamedTemporaryFile(suffix=".obj", delete=False, mode='w') as f:
        for v in mesh.vertices:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        for tri in mesh.faces:
            f.write(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}\n")
        mesh_file = f.name

    body_id = 0
    if pybullet_test():
        vis_id = p.createVisualShape(p.GEOM_MESH, fileName=mesh_file, meshScale=[1,1,1])
        body_id = p.createMultiBody(baseVisualShapeIndex=vis_id, basePosition=xyz, baseOrientation=quat)

    os.remove(mesh_file)
    return unified_object(fcl_obj, body_id, mat, bvh)


def create_sphere(xyz_rvec, scale=[1,1,1]):
    xyz = xyz_rvec[:3]
    rvec = xyz_rvec[3:]
    tf, quat = transform_to_matrix(xyz, rvec)
    mat = dp.xyzabc_to_T(xyz_rvec)
    radius = scale[0]  # uniform scale only
    sphere = fcl.Sphere(radius)
    fcl_obj = fcl.CollisionObject(sphere, tf)

    body_id = 0
    if pybullet_test():
        vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[0,0,1,1])
        body_id = p.createMultiBody(baseVisualShapeIndex=vis_id, basePosition=xyz, baseOrientation=quat)

    return unified_object(fcl_obj, body_id, mat, sphere)


class Node:
    def __init__(self, name, parent=None, local_transform=None, tags=[], target_tags=[]):
        self.name = name
        self.parent = None
        self.children = []
        self.lock = False
        self.local_transform = local_transform if local_transform is not None else np.eye(4)

        # Geometry
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


    def update_collision_object_transforms(self, gt=None):
        if gt is None:
            gt = self.get_global_transform()
        for coll_obj in self.collisions:
            new_tf = fcl_transform_from_matrix(gt @ coll_obj.mat)
            coll_obj.fcl_object.setTransform(new_tf)