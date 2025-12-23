import numpy as np
import requests
import os
import json

def inv_dh(T):
    T = np.array(T, dtype=float)
    R = np.matrix([
        [T[0,0], T[1,0], T[2,0]],
        [T[0,1], T[1,1], T[2,1]],
        [T[0,2], T[1,2], T[2,2]]
    ])
    S = -np.matmul(R, [[T[0,3]], [T[1,3]], [T[2,3]]])

    return np.matrix([
        [R[0,0], R[0,1], R[0,2], S[0,0]],
        [R[1,0], R[1,1], R[1,2], S[1,0]],
        [R[2,0], R[2,1], R[2,2], S[2,0]],
        [0, 0, 0, 1]    
    ])
        
def rmat_to_quat(rmat):
    rmat = np.array(rmat, dtype=float)
    m00, m01, m02 = rmat[0,0], rmat[0,1], rmat[0,2]
    m10, m11, m12 = rmat[1,0], rmat[1,1], rmat[1,2]
    m20, m21, m22 = rmat[2,0], rmat[2,1], rmat[2,2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m21 - m12) / S
        qy = (m02 - m20) / S
        qz = (m10 - m01) / S
    elif (m00 > m11) and (m00 > m22):
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2
        qw = (m21 - m12) / S
        qx = 0.25 * S
        qy = (m01 + m10) / S
        qz = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2
        qw = (m02 - m20) / S
        qx = (m01 + m10) / S
        qy = 0.25 * S
        qz = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2
        qw = (m10 - m01) / S
        qx = (m02 + m20) / S
        qy = (m12 + m21) / S
        qz = 0.25 * S
    return [qx, qy, qz, qw]

def quat_to_rmat(quat):
    qx, qy, qz, qw = quat
    qx2, qy2, qz2 = qx*qx, qy*qy, qz*qz
    return [
        [1 - 2*qy2 - 2*qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx2 - 2*qz2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx2 - 2*qy2]
    ]

def quat_dot(quat1, quat2):
    q1 = np.array(quat1)
    q2 = np.array(quat2)
    return float(np.dot(q1, q2))

def quat_slerp(quat1, quat2, t):
    q1 = np.array(quat1, dtype=float)
    q2 = np.array(quat2, dtype=float)
    dot = np.clip(np.dot(q1, q2), -1.0, 1.0)
    theta = np.arccos(dot)
    if abs(np.sin(theta)) < 1e-5:
        return quat1
    sin_theta = np.sin(theta)
    s1 = np.sin((1 - t) * theta) / sin_theta
    s2 = np.sin(t * theta) / sin_theta
    result = s1 * q1 + s2 * q2
    return result.tolist()

def quat_mul(quat1, quat2):
    x1, y1, z1, w1 = quat1
    x2, y2, z2, w2 = quat2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [x, y, z, w]


def abc_to_rmat(abc):
    """
    abc: [a,b,c] = axis * angle_deg
    Returns: 3x3 numpy.ndarray of dtype float
    """
    v = np.asarray(abc, dtype=float)
    angle_deg = np.linalg.norm(v)

    # 0° → identity
    if angle_deg < 1e-12:
        return np.eye(3, dtype=float)

    axis = v / angle_deg
    ux, uy, uz = axis.tolist()

    theta = np.radians(angle_deg)
    ct = float(np.cos(theta))
    st = float(np.sin(theta))
    vt = 1.0 - ct

    R = np.array([
        [ct + ux*ux*vt,     ux*uy*vt - uz*st, ux*uz*vt + uy*st],
        [uy*ux*vt + uz*st,  ct + uy*uy*vt,    uy*uz*vt - ux*st],
        [uz*ux*vt - uy*st,  uz*uy*vt + ux*st, ct + uz*uz*vt]
    ], dtype=float)

    return R

def rmat_to_abc(R):
    """
    R: 3x3 matrix (list or ndarray)
    Returns: [a,b,c] as Python floats
    Our convention: abc = axis * angle_deg
    """
    R = np.asarray(R, dtype=float)

    # --- 0) re-orthogonalize ---
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    # --- 1) angle from trace ---
    trace = float(np.trace(R))
    cos_theta = (trace - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    theta = float(np.arccos(cos_theta))   # radians

    # --- 2) 0° → abc = [0,0,0] ---
    if theta < 1e-12:
        return [0.0, 0.0, 0.0]

    eps = 1e-8

    # --- 3) 180° case ---
    if abs(theta - np.pi) < 1e-6:
        xx = (R[0, 0] + 1.0) / 2.0
        yy = (R[1, 1] + 1.0) / 2.0
        zz = (R[2, 2] + 1.0) / 2.0

        if xx >= yy and xx >= zz:
            x = np.sqrt(max(xx, 0.0))
            y = (R[0, 1] + R[1, 0]) / (4.0 * x) if x > eps else 0.0
            z = (R[0, 2] + R[2, 0]) / (4.0 * x) if x > eps else 0.0
        elif yy >= zz:
            y = np.sqrt(max(yy, 0.0))
            x = (R[0, 1] + R[1, 0]) / (4.0 * y) if y > eps else 0.0
            z = (R[1, 2] + R[2, 1]) / (4.0 * y) if y > eps else 0.0
        else:
            z = np.sqrt(max(zz, 0.0))
            x = (R[0, 2] + R[2, 0]) / (4.0 * z) if z > eps else 0.0
            y = (R[1, 2] + R[2, 1]) / (4.0 * z) if z > eps else 0.0

        axis = np.array([x, y, z], dtype=float)
        n = np.linalg.norm(axis)
        if n < eps:
            return [0.0, 0.0, 0.0]
        axis = axis / n
    else:
        # --- 4) general case ---
        denom = 2.0 * np.sin(theta)
        if abs(denom) < 1e-12:
            return [0.0, 0.0, 0.0]

        ux = (R[2, 1] - R[1, 2]) / denom
        uy = (R[0, 2] - R[2, 0]) / denom
        uz = (R[1, 0] - R[0, 1]) / denom
        axis = np.array([ux, uy, uz], dtype=float)

        n = np.linalg.norm(axis)
        if n < 1e-12:
            return [0.0, 0.0, 0.0]
        axis = axis / n

    angle_deg = float(np.degrees(theta))
    abc = (axis * angle_deg).tolist()

    # ensure python floats
    return [float(abc[0]), float(abc[1]), float(abc[2])]

"""
def abc_to_rmat(abc):
    ax, ay, az = abc
    mag = np.linalg.norm([ax, ay, az])
    theta = np.radians(mag)
    if mag > 1e-5:
        ux, uy, uz = ax/mag, ay/mag, az/mag
    else:
        ux = uy = uz = 0.0
    ct, st = np.cos(theta), np.sin(theta)
    one_ct = 1 - ct
    return [
        [ct + ux*ux*one_ct,     ux*uy*one_ct - uz*st, ux*uz*one_ct + uy*st],
        [uy*ux*one_ct + uz*st,  ct + uy*uy*one_ct,    uy*uz*one_ct - ux*st],
        [uz*ux*one_ct - uy*st,  uz*uy*one_ct + ux*st, ct + uz*uz*one_ct]
    ]

def rmat_to_abc(rmat):
    rmat = np.array(rmat, dtype=float)
    trace = np.trace(rmat)
    cos_theta = np.clip((trace - 1)/2, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-7:
        return [0.0, 0.0, 0.0]
    st = np.sin(theta)
    if abs(theta - np.pi) > 1e-6:
        x = (rmat[2,1] - rmat[1,2])/(2*st)
        y = (rmat[0,2] - rmat[2,0])/(2*st)
        z = (rmat[1,0] - rmat[0,1])/(2*st)
    else:
        x = np.sqrt(max(rmat[0,0] + 1, 0) / 2)
        y = np.sqrt(max(rmat[1,1] + 1, 0) / 2)
        z = np.sqrt(max(rmat[2,2] + 1, 0) / 2)

        if rmat[1,0]<0: y = -y
        if rmat[2,0]<0: z = -z
    return [float(a * np.degrees(theta)) for a in [x, y, z]]
"""
def T_to_xyzabc(T):
    T = np.array(T, dtype=float)
    abc = rmat_to_abc(T[:3,:3])
    return [float(T[0,3]), float(T[1,3]), float(T[2,3])] + abc

def xyzabc_to_T(xyzabc):
    T = np.eye(4)
    T[:3,:3] = abc_to_rmat(xyzabc[3:])
    T[0,3], T[1,3], T[2,3] = xyzabc[0], xyzabc[1], xyzabc[2]
    nested = T.tolist()
    return [[float(x) for x in row] for row in nested]


def rvec_to_abc(rvec):
    return np.degrees(rvec).tolist()    


def abc_to_rvec(abc):
    return np.radians(abc).tolist()


def transform_pose(offset=[0, 0, 0, 0, 0, 0], from_frame=[0,0,0,0,0,0], to_frame=[0,0,0,0,0,0]):
    T_pose = np.array(xyzabc_to_T(offset))
    T_from = np.array(xyzabc_to_T(from_frame))
    T_to   = np.array(xyzabc_to_T(to_frame))
    R_to, t_to = T_to[:3,:3], T_to[:3,3]
    R_to_inv = R_to.T
    t_to_inv = -R_to_inv @ t_to
    T_to_inv = np.eye(4)
    T_to_inv[:3,:3], T_to_inv[:3,3] = R_to_inv, t_to_inv
    T_result = T_to_inv @ T_from @ T_pose
    return T_to_xyzabc(T_result)


def rotate_abc(abc, axis=[0,0,1], angle=0, local=False):
    T = np.array(abc_to_rmat(abc))
    axis = np.array(axis, dtype=float)
    if local:
        axis = T @ axis
    axis /= np.linalg.norm(axis)
    R = np.array(abc_to_rmat(list(axis*angle)))
    RT = R @ T
    return rmat_to_abc(RT)


def align_abc(abc, align=[0, 0, 1], axis=[0, 0, 1], fix=None):
    """
    Rotate the pose so that its local X/Y/Z axis aligns with `axis`.
    - abc:    [ax, ay, az] axis-angle in degrees
    - align:  initial direction in local frame
    - axis:   target direction in world frame
    - fix: arbitrary vector in local frame so the rotation fixs it (or projects it at most)
    Returns a new [ax, ay, az] so that the requested local axis now points along `axis`.
    """
    # 1) Build current rotation matrix
    current_rmat = np.array(abc_to_rmat(abc))
    
    align = np.array(align)

    #defining auxilary vectors a,b,c, all living in globla coorfinate frame
    a = align
    a = np.array(np.matmul(current_rmat, a).flatten().tolist())
    a = a / np.linalg.norm(a)

    #prepare parameteres for the case if fix exist
    if fix is not None:
        fix = np.array(fix)

        fix = fix - align * np.dot(fix, align)
        if(np.isclose(np.linalg.norm(fix),0.0)):
            fix = None
        else:
            fix = fix / np.linalg.norm(fix) 
            c = np.array(np.matmul(current_rmat, fix).flatten().tolist())

    # 3) Normalize the target axis
    b = np.array(axis, dtype=float)
    b = b / np.linalg.norm(b)

    # 4) Compute dot and clamp
    dot = np.clip(np.dot(a, b), -1.0, 1.0)

    # 5) If already aligned
    if np.isclose(dot, 1.0):
        return abc

    # 6) If anti-aligned, pick any orthogonal vector & 180° rotation
    if np.isclose(dot, -1.0):
        if abs(a[0]) < abs(a[1]):
            orth = np.array([-a[2], 0.0, a[0]])
        else:
            orth = np.array([0.0, -a[2], a[1]])
        orth = orth / np.linalg.norm(orth)
        rot_deg = 180.0

    else:
        # 7) General case: axis = cross(a,b), angle = arccos(dot)
        orth = np.cross(a, b)
        orth = orth / np.linalg.norm(orth)
        rot_deg = np.degrees(np.arccos(dot))

    # 8) Apply rotation around `orth`
    result = rotate_abc(abc, orth.tolist(), rot_deg, local=False)

    # 9) in case fixture mechanism exist one extra step is needed
    if fix is not None:
        proj_c = c - b * np.dot(c, b)
        proj_c = proj_c / np.linalg.norm(proj_c)
        return align_abc(result, fix, proj_c, fix=None)

    return result


def robot_to_frame(xyzabc, aux=[0, 0], aux_dir=[[1, 0, 0], [0, 0, 0]], base_in_world=[0, 0, 0, 0, 0, 0], frame_in_world = [0, 0, 0, 0, 0, 0]):
    # adjust xyzabcde
    xyzabc = list(xyzabc+ [0 for _ in range(6)])[0:6]
    aux_offset = aux[0]*np.append(aux_dir[0],[0, 0, 0]) + aux[1]*np.append(aux_dir[1],[0, 0, 0])

    # xyzabc
    xyzabc_base = np.array(xyzabc) + aux_offset
    xyzabc_world = transform_pose(xyzabc_base, from_frame=base_in_world, to_frame=[0,0,0,0,0,0])
    xyzabc_frame = transform_pose(xyzabc_world, from_frame=[0,0,0,0,0,0], to_frame=frame_in_world)
    return xyzabc_frame

def frame_to_robot(xyzabc, aux=[0, 0], aux_dir=[[1, 0, 0], [0, 0, 0]], base_in_world=[0, 0, 0, 0, 0, 0], frame_in_world = [0, 0, 0, 0, 0, 0]):
    # adjust xyzabcde
    xyzabc = list(xyzabc+ [0 for _ in range(6)])[0:6]
    aux_offset = aux[0]*np.append(aux_dir[0],[0, 0, 0]) + aux[1]*np.append(aux_dir[1],[0, 0, 0])

    # xyzabc
    xyzabc_world = transform_pose(xyzabc, from_frame=frame_in_world, to_frame=[0,0,0,0,0,0])
    xyzabc_base = np.array(transform_pose(xyzabc_world, from_frame=[0,0,0,0,0,0], to_frame=base_in_world))
    xyzabc_robot = xyzabc_base - aux_offset
    return xyzabc_robot.tolist()


class Pose:
    def __init__(self, name=None, pose=None, anchors={}):
        """
        name:    unique identifier
        pose:    [x,y,z,a,b,c] local transform relative to parent
        parent:  who we are attached to
        anchors: anchor-name -> [x,y,z,a,b,c]
        """
        self.name = name
        self.local = {
            "xyzabc": list(pose or [0, 0, 0, 0, 0, 0]),
            "T": np.array(xyzabc_to_T(pose or [0, 0, 0, 0, 0, 0])),
        }

        self.parent = {
            "parent_solid": None,
            "parent_anchor": None,
            "child_anchor": None,
            "offset": None,
        }

        # children grouped by parent anchor name
        self.children = {anchor: [] for anchor in anchors.keys()}
        self.anchors = dict(anchors)


    # -----------------------------------------------------------
    # NEW: Mark this solid and entire subtree dirty + clear cache
    # -----------------------------------------------------------
    def mark_subtree_dirty(self):
        stack = [self]
        while stack:
            node = stack.pop()
            node._pose_flag = True
            node._world_T = None        # invalidate cached world transform

            # propagate to children
            for child_list in node.children.values():
                for entry in child_list:
                    stack.append(entry["child_solid"])


    # ---------- basic pose ops ----------
    def get_local(self, anchor):
        if anchor not in self.anchors:
            raise KeyError(f"anchor '{anchor}' not found on {self.name}")
        return list(self.anchors[anchor])


    def pose(self, anchor=None, in_frame=None, pose=None, offset=[0, 0, 0, 0, 0, 0]):
        """
        Get this object's pose expressed in another frame.
        """

        # build transform to world
        def to_world(node, anc=None):
            T = np.eye(4)
            if anc is not None:
                T = T @ np.array(xyzabc_to_T(node.get_local(anc)))

            cur = node
            while cur is not None:
                T = np.array(cur.local["T"]) @ T
                cur = cur.parent["parent_solid"]
            return T

        # in_frame == self
        if in_frame is self:
            if anchor is not None:
                T_local = np.array(xyzabc_to_T(self.get_local(anchor)))
            else:
                T_local = np.eye(4)

            if pose is not None:
                T_local = T_local @ np.array(xyzabc_to_T(pose))
            if offset is not None:
                T_local = T_local @ np.array(xyzabc_to_T(offset))
            return T_to_xyzabc(T_local)

        # world transform of this solid
        T_self_world = to_world(self, anchor)

        if pose is not None:
            T_self_world = T_self_world @ np.array(xyzabc_to_T(pose))
        if offset is not None:
            T_self_world = T_self_world @ np.array(xyzabc_to_T(offset))

        if in_frame is None:
            return T_to_xyzabc(T_self_world)

        # express in another solid's frame
        T_frame_world = to_world(in_frame)
        T_frame_self = np.linalg.inv(T_frame_world) @ T_self_world
        return T_to_xyzabc(T_frame_self)


    # ---------- attach (child-side) ----------
    def attach_to(
        self,
        parent,
        parent_anchor,
        child_anchor,
        *,
        offset=[0, 0, 0, 0, 0, 0],
        offset_frame="parent",
    ):
        """
        Attach this Pose to a parent solid by mating anchors.
        """

        # parent anchor transform in parent frame
        T_pa_local = np.array(xyzabc_to_T(parent.get_local(parent_anchor)))

        # child anchor transform in child frame
        T_ca_local = np.array(xyzabc_to_T(self.get_local(child_anchor)))

        # child pose relative to parent (anchors aligned)
        T_child_local = T_pa_local @ np.linalg.inv(T_ca_local)

        # apply offset
        if offset is not None:
            T_off = np.array(xyzabc_to_T(offset))
            if offset_frame == "parent":
                T_child_local = T_pa_local @ T_off @ np.linalg.inv(T_ca_local)
            elif offset_frame == "child":
                T_child_local = T_child_local @ T_off
            else:
                raise ValueError("offset_frame must be 'parent' or 'child'")

        child_local_xyzabc = T_to_xyzabc(T_child_local)

        # remove from old parent
        if self.parent["parent_solid"] is not None:
            old = self.parent["parent_solid"]
            anchor = self.parent["parent_anchor"]
            old.children[anchor] = [
                c for c in old.children[anchor] if c["child_solid"] is not self
            ]

        # set new parent
        self.parent = {
            "parent_solid": parent,
            "parent_anchor": parent_anchor,
            "child_anchor": child_anchor,
            "offset": offset,
        }
        parent.children[parent_anchor].append({
            "child_solid": self,
            "child_anchor": child_anchor,
            "offset": offset,
        })

        # update local transform
        self.local = {
            "xyzabc": list(child_local_xyzabc),
            "T": np.array(T_child_local),
        }

        # -----------------------------------------------------------
        # CRITICAL: mark this solid + subtree AND parent chain dirty
        # -----------------------------------------------------------
        self._pose_flag = True
        self._world_T = None
        self.mark_subtree_dirty()

        # parent must also be marked dirty, or DFS may skip recomputation
        parent._pose_flag = True
        parent._world_T = None

        return child_local_xyzabc



class Solid(Pose):
    def __init__(self, name=None, pose=None, anchors={}, **kwargs):
        super().__init__(name, pose, anchors)

        # initialize runtime fields for Workspace optimized DFS
        self._pose_flag = False      # whether this solid changed this pass
        self._world_T = None         # cached world transform

        # attach any extra attributes
        for key, value in kwargs.items():
            setattr(self, key, value)


