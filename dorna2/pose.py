import numpy as np
import copy

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


def transform_pose(xyzabc, from_frame=[0,0,0,0,0,0], to_frame=[0,0,0,0,0,0]):
    T_pose = np.array(xyzabc_to_T(xyzabc))
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
    def __init__(self, name, pose=None, parent=None, anchors=None):
        """
        name:    unique identifier
        pose:    [x,y,z,a,b,c] local to parent (defaults to identity)
        parent:  Pose or None
        anchors: dict{name:[x,y,z,a,b,c]} OR list[(name, [x,y,z,a,b,c]), ...]
        """
        self.name = name
        self.local_pose = list(pose or [0,0,0,0,0,0])
        self.parent = parent
        self.children = []
        self.anchors = {}

        if parent is not None:
            parent.children.append(self)

        if anchors:
            self.add_anchors(anchors)

    # ---------- anchors ----------
    def add_anchor(self, name, xyzabc_local):
        """Register/overwrite an anchor in this node's local frame."""
        self.anchors[name] = list(xyzabc_local)

    def add_anchors(self, anchors):
        """Accepts dict or list of (name, xyzabc)."""
        if isinstance(anchors, dict):
            for k, v in anchors.items():
                self.add_anchor(k, v)
        elif isinstance(anchors, (list, tuple)):
            for item in anchors:
                if not (isinstance(item, (list, tuple)) and len(item) == 2):
                    raise ValueError("anchors list must contain (name, xyzabc) pairs")
                name, xyzabc = item
                self.add_anchor(name, xyzabc)
        else:
            raise TypeError("anchors must be dict or list of (name, xyzabc)")


    # ---------- basic pose ops ----------
    def get_local(self, anchor):
        if anchor not in self.anchors:
            raise KeyError(f"anchor '{anchor}' not found on {self.name}")
        return list(self.anchors[anchor])


    def set_pose(self, pose):
        self.local_pose = list(pose)


    def pose(self, anchor=None, pose=None, in_frame=None):
        """
        Get this object's pose expressed in another frame.

        anchor:    if given, use this anchor's pose (local to this object)
        pose:      if given and anchor=None, interpret as local pose in this object
        in_frame:  Pose object to express pose in (None = world, self = local)
        """
        # Step 1: starting pose in this object's frame
        if anchor is not None:
            if anchor not in self.anchors:
                raise KeyError(f"anchor '{anchor}' not found on {self.name}")
            local_pose = list(self.anchors[anchor])
        elif pose is not None:
            local_pose = list(pose)
        else:
            local_pose = list(self.local_pose)

        # helper to build chain from root to this node
        def chain_to_root(node):
            chain = []
            while node is not None:
                chain.append(node)
                node = node.parent
            return chain[::-1]  # root → self order

        # helper to compute transform from root to node
        def T_root_to(node):
            T = np.eye(4)
            if node.parent is None:
                return np.array(xyzabc_to_T(node.local_pose))
            chain = chain_to_root(node)
            for idx, n in enumerate(chain):
                if idx == 0:
                    T = np.array(xyzabc_to_T(n.local_pose))
                else:
                    T = T @ np.array(xyzabc_to_T(n.local_pose))
            return T

        # if asking in_frame == self
        if in_frame is self:
            return local_pose

        # if no in_frame, default is world
        if in_frame is None:
            # standard: root/world-relative
            T_self_world = T_root_to(self)
            if anchor is not None or pose is not None:
                T_self_world = T_self_world @ np.array(xyzabc_to_T(local_pose))
            return T_to_xyzabc(T_self_world)

        # compute direct relative transform between two nodes
        chain_self = chain_to_root(self)
        chain_frame = chain_to_root(in_frame)

        # find common ancestor
        common_ancestor = None
        for n1, n2 in zip(chain_self, chain_frame):
            if n1 is n2:
                common_ancestor = n1
            else:
                break
        if common_ancestor is None:
            raise ValueError("No common ancestor found in pose graph")

        # transform from common ancestor to self
        idx_ca_self = chain_self.index(common_ancestor)
        T_ca_self = np.eye(4)
        for n in chain_self[idx_ca_self+1:]:
            T_ca_self = T_ca_self @ np.array(xyzabc_to_T(n.local_pose))
        if anchor is not None or pose is not None:
            T_ca_self = T_ca_self @ np.array(xyzabc_to_T(local_pose))

        # transform from common ancestor to in_frame
        idx_ca_frame = chain_frame.index(common_ancestor)
        T_ca_frame = np.eye(4)
        for n in chain_frame[idx_ca_frame+1:]:
            T_ca_frame = T_ca_frame @ np.array(xyzabc_to_T(n.local_pose))

        # relative transform: in_frame → self
        T_frame_self = np.linalg.inv(T_ca_frame) @ T_ca_self
        return T_to_xyzabc(T_frame_self)
        

    def find(self, name):
        if self.name == name:
            return self
        for c in self.children:
            hit = c.find(name)
            if hit is not None:
                return hit
        return None

    def __repr__(self):
        return f"<Pose {self.name}: local_pose={self.local_pose}, anchors={list(self.anchors.keys())}>"

    # ---------- graph helpers ----------
    def detach(self):
        """Remove from current parent (if any)."""
        if self.parent is not None and self in self.parent.children:
            self.parent.children.remove(self)
        self.parent = None

    # ---------- attach (child-side) ----------
    def attach_to(
        self,
        parent,
        parent_anchor,
        child_anchor,
        *,
        align=('z','z'),
        flip=False,
        offset=[0,0,0,0,0,0],    # [dx,dy,dz, ax,ay,az] (mm, deg)
        offset_frame='parent',   # 'parent' or 'child'
        preview=False
    ):
        """
        Place THIS pose (child) onto 'parent' by mating anchors.

        Default behavior (no offset, align=('z','z'), flip=False):
        - Child's 'child_anchor' frame is made coincident with the parent's 'parent_anchor' frame
          (same position & orientation).

        Options:
        - align=(child_axis, parent_axis): 'x'|'y'|'z' to align a child axis to a parent axis.
        - flip=True: align to the negative of the parent axis.
        - offset=[dx,dy,dz, ax,ay,az]: applied AFTER mating, in 'offset_frame' coords.
        - offset_frame: 'parent' (applied in parent anchor frame) or 'child' (applied in child anchor frame).
        - preview=True: compute and return the would-be local pose without mutating the graph.
        """
        # --- lookups & build transforms ---
        T_parent_world = np.array(xyzabc_to_T(parent.pose()))
        T_pa = np.array(xyzabc_to_T(parent.get_local(parent_anchor)))  # parent anchor in parent frame
        T_ca = np.array(xyzabc_to_T(self.get_local(child_anchor)))     # child anchor in child frame

        # --- alignment tweak: R_extra (world) ---
        R_extra = np.eye(4)
        if align is not None:
            ax_map = {'x': np.array([1.,0.,0.]),
                      'y': np.array([0.,1.,0.]),
                      'z': np.array([0.,0.,1.])}
            c_ax = ax_map[align[0].lower()]
            p_ax = ax_map[align[1].lower()]

            R_parent_anchor = (T_parent_world @ T_pa)[:3,:3]
            R_child_anchor  = T_ca[:3,:3]

            c_axis_w = R_child_anchor @ c_ax
            p_axis_w = R_parent_anchor @ p_ax
            if flip:
                p_axis_w = -p_axis_w

            # normalize
            c_axis_w = c_axis_w / np.linalg.norm(c_axis_w)
            p_axis_w = p_axis_w / np.linalg.norm(p_axis_w)

            dot = float(np.clip(np.dot(c_axis_w, p_axis_w), -1.0, 1.0))
            if dot >= 1.0 - 1e-12:
                R_world = np.eye(3)
            elif dot <= -1.0 + 1e-12:
                # rotate 180° about any axis ⟂ c_axis_w
                tmp = np.array([1.0, 0.0, 0.0])
                if abs(np.dot(tmp, c_axis_w)) > 0.9:
                    tmp = np.array([0.0, 1.0, 0.0])
                k = np.cross(c_axis_w, tmp)
                k = k / np.linalg.norm(k)
                R_world = np.array(abc_to_rmat((k * 180.0).tolist()))
            else:
                k = np.cross(c_axis_w, p_axis_w)
                k_norm = np.linalg.norm(k)
                if k_norm < 1e-12:
                    R_world = np.eye(3)
                else:
                    k /= k_norm
                    ang = np.degrees(np.arccos(dot))
                    R_world = np.array(abc_to_rmat((k * ang).tolist()))

            R_extra[:3,:3] = R_world

        # --- base child world pose (anchors coincident, aligned) ---
        # T_child_world = T_parent_world * T_pa * R_extra * inv(T_ca)
        T_child_world = T_parent_world @ T_pa @ R_extra @ np.linalg.inv(T_ca)

        # --- apply offset AFTER mating ---
        if offset is not None:
            dx,dy,dz, ax,ay,az = [float(v) for v in offset]
            T_off = np.array(xyzabc_to_T([dx,dy,dz, ax,ay,az]))

            if offset_frame == 'parent':
                # apply in parent-anchor frame: insert after T_parent_world@T_pa@R_extra
                T_child_world = T_parent_world @ T_pa @ R_extra @ T_off @ np.linalg.inv(T_ca)
            elif offset_frame == 'child':
                # apply in child frame after mating
                T_child_world = T_child_world @ T_off
            else:
                raise ValueError("offset_frame must be 'parent' or 'child'")

        # --- convert to child's LOCAL pose w.r.t parent ---
        # T_child_world = T_parent_world @ T_child_local  => T_child_local = inv(T_parent_world) @ T_child_world
        T_child_local = np.linalg.inv(T_parent_world) @ T_child_world
        child_local_xyzabc = T_to_xyzabc(T_child_local)

        if preview:
            return child_local_xyzabc

        # mutate graph
        if self.parent is not None and self in self.parent.children:
            self.parent.children.remove(self)
        self.parent = parent
        parent.children.append(self)
        self.local_pose = child_local_xyzabc
        return child_local_xyzabc


