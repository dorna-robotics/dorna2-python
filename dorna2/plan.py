import numpy as np
import time 
import sys
import fcl

from dorna2 import Dorna
from dorna2.pathGen import pathGen
from dorna2.urdf import UrdfRobot
import dorna2.node as node
import dorna2.pose as dp
#import pybullet as p



# ---- helpers from your kinematics object (names match your methods) ----
# Ti_r_world(i, theta=...)  -> 4x4 world transform of joint frame i
# fw_base(theta)            -> 4x4 world transform of TCP/EE (optional, for convenience)

def _origin_and_axis_world(kin, i, theta):
    """Return origin o_i (3,) and z-axis z_i (3,) of joint i, both in world frame."""
    Ti = kin.Ti_r_world(theta=theta, i=i)          # 4x4
    Ri = Ti[:3, :3]
    oi = Ti[:3, 3]
    zi = Ri @ np.array([0.0, 0.0, 1.0])            # DH: joint axis = +Z_i
    return oi, zi

def jacobian_point(kin, link_idx, p_world, theta_deg):
    """
    Spatial point Jacobian at a world-space point p_world rigidly attached to `link_idx`.
    Returns: {'linear': Jv (3xn), 'angular': Jw (3xn)}
    """
    n = 6
    theta = [np.radians(j) for j in theta_deg]

    Jv = np.zeros((3, n))
    Jw = np.zeros((3, n))

    # Only joints 1..link_idx affect the pose of link_idx in a serial chain
    # (Assuming your frames are indexed like your Ti_r_world: i=1..6 are actuated)
    for k in range(1, n+1):
        if k > link_idx:
            # joints after the link do not move this point (point is on link_idx)
            continue

        ok, zk = _origin_and_axis_world(kin, k, theta)

        # revolute: angular = z_k ; linear = z_k × (p - o_k)
        Jw[:, k-1] = zk
        Jv[:, k-1] = np.cross(zk, (p_world - ok))

    return {'linear': Jv, 'angular': Jw}

def jacobian_ee_rotation(kin, theta_deg):
    """
    Angular Jacobian of the end-effector orientation in world.
    Returns Jw (3xn).
    """
    n = 6
    theta = [np.radians(j) for j in theta_deg]
    Jw = np.zeros((3, n))
    for k in range(1, n+1):
        ok, zk = _origin_and_axis_world(kin, k, theta)
        Jw[:, k-1] = zk
    return Jw


def correct_pose_kinematic(
    joint,
    scene=[], load=[],
    base_in_world=[0,0,0,0,0,0],
    frame_in_world=[0,0,0,0,0,0],
    keep_ee_rotation=False,
    keep_rot_weight=1000.0,
    lam=1e-3,
    beta=0.9,
    target_slop=1e-3,
    max_iters=10,
    max_step_norm=0.1,
    line_search=True
):
    sim = Simulation("tmp_pose")   # your class
    n = sim.robot.num_dofs         # or len(joint)

    q_deg = np.array(joint, dtype=float)

    # ----- mount scene -----
    all_visuals = []
    all_objects = []
    dynamic_objects = []

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
        cdata = fcl.CollisionData()
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
            p = np.array(c.contact_point)      # world 3-vector
            n = np.array(c.normal)             # world 3-vector (points from o1 to o2)
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

            contacts.append({
                'p': p, 'n': n / max(1e-12, np.linalg.norm(n)),
                'depth': depth,          # may be 0; we’ll treat as interpenetrating if distance < slop
                'linkA': linkA, 'linkB': linkB
            })
        return contacts


    # ----- solve loop -----
    q = np.array(joint, dtype=float).copy()
    set_pose(q)

    debug = {'iters': []}

    for it in range(max_iters):
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
            JvA = jacobian_point(sim.dorna.kinematic, ct['linkA'].joint_index, p, q_deg)['linear']          # (3,n)
            if ct['linkB'] is None:
                JvB = 0.0
            else:
                JvB = jacobian_point(,sim.dorna.kinematic, ct['linkB'].joint_index, p, q_deg)['linear']      # (3,n)

            Jn = n @ (JvA - (JvB if isinstance(JvB, np.ndarray) else 0))
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

        # Clamp step size for stability
        norm = np.linalg.norm(dq)
        if norm > max_step_norm:
            dq = dq * (max_step_norm / max(norm, 1e-12))

        # Optional line search to ensure improvement (reduce total “penetration” proxy)
        if line_search:
            def score():
                # Sum of positive depths; if none available, count number of contacts
                s = 0.0
                for ct in get_contacts():
                    s += max(target_slop, ct['depth'] if ct['depth'] > 0 else target_slop)
                return s

            cur_score = score()
            alpha = 1.0
            accepted = False
            for _ in range(6):
                q_try = q + alpha * dq
                set_pose(q_try)
                new_score = score()
                if new_score <= 0.95 * cur_score:
                    q = q_try
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                # take a tiny step anyway to avoid stalls
                q = q + 0.2 * dq
                set_pose(q)
        else:
            q = q + dq
            set_pose(q)

        q_deg = q.copy()


        debug['iters'].append({'num_contacts': len(contacts), 'step_norm': float(np.linalg.norm(dq))})

        # Early out if everything looks clear
        if len(get_contacts()) == 0:
            break

    return q, {'sim': sim, 'visuals': all_visuals, 'iters': debug['iters']}
