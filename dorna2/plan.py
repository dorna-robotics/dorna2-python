import numpy as np
import time 
import sys
import fcl

from dorna2 import Dorna
from dorna2.path_gen import path_gen, path
from dorna2.simulation import simulation
from dorna2.urdf import urdf_robot
import dorna2.node as node
import dorna2.pose as dp
#import pybullet as p

# ---- helpers from your kinematics object (names match your methods) ----
# Ti_r_world(i, theta=...)  -> 4x4 world transform of joint frame i
# fw_base(theta)            -> 4x4 world transform of TCP/EE (optional, for convenience)

def _origin_and_axis_world(kin, i, joint):
    """Return origin o_i (3,) and z-axis z_i (3,) of joint i, both in world frame."""
    Ti = kin.Ti_r_world(joint=joint, i=i-1)          # 4x4
    Ri = Ti[:3, :3]
    oi = Ti[:3, 3] / 1000.0

    zi = Ri @ np.array([0.0, 0.0, 1.0])            # DH: joint axis = +Z_i

    return oi, zi

def _vec3(x):
    x = np.asarray(x)
    if x.ndim > 1:
        x = x.squeeze()
    if x.shape[0] == 4:            # drop homogeneous w if present
        x = x[:3]
    return x.astype(float).reshape(3,)

def jacobian_point(kin, link_idx, p_world, joint):
    """
    Spatial point Jacobian at a world-space point p_world rigidly attached to `link_idx`.
    Returns: {'linear': Jv (3xn), 'angular': Jw (3xn)}
    """
    n = 6

    Jv = np.zeros((3, n))
    Jw = np.zeros((3, n))

    # Only joints 1..link_idx affect the pose of link_idx in a serial chain
    # (Assuming your frames are indexed like your Ti_r_world: i=1..6 are actuated)
    #print("p",p_world)
    for k in range(1, n+1):
        if k > link_idx:
            # joints after the link do not move this point (point is on link_idx)
            continue

        ok, zk = _origin_and_axis_world(kin, k, joint)
        ok = _vec3(ok)
        zk = _vec3(zk)
        p = _vec3(p_world)

        deg2rad = np.pi / 180.0
        #print(k-1, ok,zk)
        # revolute: angular = z_k ; linear = z_k × (p - o_k)
        Jw[:, k-1] = zk * deg2rad
        Jv[:, k-1] = np.cross(zk, (p - ok)) * deg2rad
    #print("Jv",Jv)

    return {'linear': Jv, 'angular': Jw}

def jacobian_ee_rotation(kin, joint):
    """
    Angular Jacobian of the end-effector orientation in world.
    Returns Jw (3xn).
    """
    deg2rad = np.pi / 180.0
 
    n = 6
    Jw = np.zeros((3, n))
    for k in range(1, n+1):
        ok, zk = _origin_and_axis_world(kin, k, joint)
        Jw[:, k-1] = zk
    return Jw * deg2rad

def link_name_to_num(s):
    if len(s) < 2:
        return 0
    c = s[1]
    return int(c) if c.isdigit() else 0

def correct_pose_kinematic(
    joint,
    scene=[], load=[], tool=[0,0,0,0,0,0],
    base_in_world=[0,0,0,0,0,0],
    frame_in_world=[0,0,0,0,0,0],
    keep_ee_rotation=False,
    keep_rot_weight=1000.0,
    lam=1e-3,
    beta=0.9,
    target_slop=1e-3,
    max_iters=10,
    max_step_norm=0.1,
    jointdot = [1,0,0,0,0,0]):
    sim = Simulation("tmp_pose")   # your class
    n = 6

    q_deg = np.array(joint[:n], dtype=float)

    # ----- mount scene -----
    all_visuals = []
    all_objects = []
    dynamic_objects = []

    sim.dorna.kinematic.set_tcp_xyzabc(tool)

    for obj in scene:
        sim.root_node.collisions.append(obj.fcl_object)
        all_objects.append(obj.fcl_object)
        all_visuals.append(obj)

    for obj in load:
        sim.robot.link_nodes["j6_link"].collisions.append(obj)
        sim.robot.all_objs.append(obj)
        sim.robot.prnt_map[id(obj.fcl_shape)] = sim.robot.link_nodes["j6_link"]

    for obj in sim.robot.all_objs:
        dynamic_objects.append(obj)
        all_objects.append(obj.fcl_object)
        all_visuals.append(obj)

    manager = fcl.DynamicAABBTreeCollisionManager()
    manager.registerObjects(all_objects)
    manager.setup()

    base_in_world_mat = sim.dorna.kinematic.xyzabc_to_mat(base_in_world)
    frame_in_world_inv = sim.dorna.kinematic.inv_dh(sim.dorna.kinematic.xyzabc_to_mat(frame_in_world))
    
    def set_pose(q):
        sim.robot.set_joint_values([0, q[0], q[1], q[2], q[3], q[4], q[5]], frame_in_world_inv @ base_in_world_mat)
        for do in dynamic_objects:
            manager.update(do.fcl_object)

    def get_contacts():
        cdata = fcl.CollisionData(
                    request=fcl.CollisionRequest(
                        enable_contact=True,
                        num_max_contacts=1,         # or higher if you want
                        enable_cost=False
                    )
                )
        def _cb(o1, o2, cdata):
            fcl.collide(o1, o2, cdata.request, cdata.result)
            return False
        manager.collide(cdata, _cb)

        contacts = []
        for c in cdata.result.contacts:
            o1, o2 = c.o1, c.o2

            # Parent lookup (like your code)
            prnt0 = sim.robot.prnt_map.get(id(o1), None)
            prnt1 = sim.robot.prnt_map.get(id(o2), None)

            num_parents = (prnt0 is not None) + (prnt1 is not None)
            if num_parents == 0:
                continue

            # filter adjacent/self adjacency if needed
            if num_parents == 2:
                if prnt0.parent == prnt1 or prnt1.parent == prnt0 or prnt0 == prnt1:
                    continue

            # Contact fields from FCL
            # Note: not all FCL pipelines fill penetration_depth reliably; fall back to small value.
            p = np.array(c.pos)      # world 3-vector
            n = np.array(c.normal)   # world 3-vector (points from o1 to o2)

            depth = getattr(c, 'penetration_depth', 0.0)
            # Decide which one is the robot link "A" we will move positively against normal.
            # We want normal to point from A toward B.
            linkA = None
            linkB = None
            if prnt0 is not None and prnt1 is None:
                linkA, linkB = prnt0, None                      # environment
                # normal currently points from o1->o2; if o1 is A, keep as is
                # (A gets pushed along +n)
            elif prnt0 is None and prnt1 is not None:
                linkA, linkB = prnt1, None
                n = -n                                          # flip so n points from A to B
            else:
                # self-collision: both robot
                linkA, linkB = prnt0, prnt1
                # ensure n points from A toward B; if not, flip
                # heuristic: we assume o1 belongs to linkA; if not, flip
                if prnt0 != linkA:
                    n = -n

            print("link: ", linkA.name, " depth: ",depth , " o1,o2: ", id(o1), id(o2))

            contacts.append({
                                'p': p, 'n': n / max(1e-12, np.linalg.norm(n)),
                                'depth': depth,          # may be 0; we’ll treat as interpenetrating if distance < slop
                                'linkA': linkA, 'linkB': linkB
            })
            break
        return contacts


    # ----- solve loop -----
    q = np.array(joint, dtype=float).copy()
    set_pose(q)
    print(q)
    debug = {'iters': []}
    debug_path = []

    for it in range(max_iters):
        print("\n iteration num: ",it)
        contacts = get_contacts()

        # Build stacked normal rows
        rows = []
        rhs  = []

        # If FCL doesn’t give depth, approximate “penetration” as target_slop along normal
        # so we still separate by ~slop when in contact.
        
        for ct in contacts:
            n = ct['n'].reshape(1,3)
            p = ct['p']
            depth = ct['depth']

            # Build relative linear Jacobian along normal

            JvA = jacobian_point(sim.dorna.kinematic, link_name_to_num(ct['linkA'].name), p, q_deg)['linear']          # (3,n)
            if ct['linkB'] is None:
                JvB = 0.0
            else:
                JvB = jacobian_point(sim.dorna.kinematic, link_name_to_num(ct['linkB'].name), p, q_deg)['linear']      # (3,n)


            Jn = n @ (JvA - (JvB if isinstance(JvB, np.ndarray) else 0))
            #print(n,Jn)
            rows.append(Jn)  # (1,n)

            # desired displacement along n:
            # if depth>0, separate a fraction of it + slop; else use slop
            desired = beta * max(target_slop, depth if depth > 0 else target_slop)
            rhs.append([desired])

        # Optional: keep end-effector rotation (soft "stay" constraint with big weight)
        if keep_ee_rotation:
            Jrot = jacobian_ee_rotation(sim.dorna.kinematic, q_deg)    # (3,n)
            if isinstance(Jrot, np.ndarray):
                # We want Δω ≈ 0 → target 0; implement as weighted rows
                W = keep_rot_weight
                rows.append(np.sqrt(W) * Jrot[0:1, :])
                rhs.append([[0.0]])
                rows.append(np.sqrt(W) * Jrot[1:2, :])
                rhs.append([[0.0]])
                rows.append(np.sqrt(W) * Jrot[2:3, :])
                rhs.append([[0.0]])

        if not rows:
            # no contacts -> done
            break

        J = np.vstack(rows)                 # (m,n)
        d = np.array(rhs, dtype=float)      # (m,1)

        # Damped least squares step: dq = J^T (J J^T + lam^2 I)^-1 d
        JJt = J @ J.T
        dq = J.T @ np.linalg.solve(JJt + (lam**2) * np.eye(JJt.shape[0]), d)

        dq = dq.ravel()


        if q.size > dq.size:
            dq = np.pad(dq, (0, q.size - dq.size), mode='constant')

        # dq should not be in the direction of the motion
        jointdot = np.array(jointdot)
        if dq.size > jointdot.size:
            jointdot = np.pad(jointdot, (0, dq.size - jointdot.size), mode='constant')

        dq = dq - jointdot * np.dot(jointdot, dq) / np.linalg.norm(jointdot)**2


        # Clamp step size for stability
        norm = np.linalg.norm(dq)
        if norm > max_step_norm:
            dq = dq * (max_step_norm / max(norm, 1e-12))

        # Optional line search to ensure improvement (reduce total “penetration” proxy)


        q = q - dq*30.0
        print(q)
        set_pose(q)

        q_deg = q.copy()


        debug['iters'].append({'num_contacts': len(contacts), 'step_norm': float(np.linalg.norm(dq))})
        debug_path.append(q)

    return q, {'sim': sim, 'visuals': all_visuals, 'iters': debug['iters'], 'debug_path':Path(debug_path)}

