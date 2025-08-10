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


# -------------------------------
# Standalone collision avoidance utilities (python-fcl)
# -------------------------------
import numpy as np
import fcl

# --------- Public API ----------

def find_worst_contact(robot_objs, scene_objs):
    """
    Returns a dict describing the worst contact between robot and scene:
        {
          'clearance': float,      # >0 = distance; <0 = penetration depth (negative)
          'p_link': (3,),          # nearest point on the robot object (world)
          'p_obs':  (3,),          # nearest point on the scene object (world)
          'normal': (3,),          # unit vector from obstacle -> link (world)
          'rob_obj': <fcl.CollisionObject>,
          'obs_obj': <fcl.CollisionObject>,
          'type': 'clearance' | 'penetration'
        }
    Returns None if neither distances nor contacts exist.
    """
    wc = _worst_clearance(robot_objs, scene_objs)      # smallest positive distance
    wp = _worst_penetration(robot_objs, scene_objs)    # largest penetration (negative clearance)

    if wc is None and wp is None:
        return None
    if wc is None:
        return wp
    if wp is None:
        return wc
    # Pick the more dangerous (smaller signed clearance)
    return wp if wp["clearance"] < wc["clearance"] else wc


def collision_avoid_one_step(
    q,
    worst,
    link_id_from_obj,
    fk_set_links,
    jacobian_point_fn,
    d_safe=0.03,
    lam=1e-2,
    alpha=0.35,
    dq_deg_cap=3.0,
    clamp_to_limits=lambda q: q,
):
    """
    Compute a single joint update that pushes the offending link point away
    from the obstacle along the contact normal.

    Parameters
    ----------
    q : (n,) ndarray
        Current joint angles (radians). For a 6-DoF arm, n=6.
    worst : dict
        Output from find_worst_contact(...).
    link_id_from_obj : callable
        Takes an fcl.CollisionObject (robot) -> returns your link_id (int or name).
    fk_set_links : callable
        fk_set_links(q) must update all robot link CollisionObjects' transforms.
        (We call this only if you do additional checks outside.)
    jacobian_point_fn : callable
        jacobian_point_fn(link_id, q, p_world) -> (3xN) position Jacobian at the *surface point* p_world.
    d_safe : float
        Safety margin in meters.
    lam : float
        Damping for DLS.
    alpha : float
        Step gain (0..1). Final Δq is alpha-scaled.
    dq_deg_cap : float
        Per-joint step cap in degrees.
    clamp_to_limits : callable
        clamp_to_limits(q) -> q_clamped (enforce joint limits).

    Returns
    -------
    q_new : (n,) ndarray
    info : dict with debug fields (gap, Δx, Δq, etc.)
    """
    if worst is None:
        return q.copy(), {"action": "no_contacts"}

    gap = d_safe - worst["clearance"]  # how much we are short of margin
    if gap <= 0.0:
        return q.copy(), {"action": "clear", "gap": gap}

    # Build task-space push
    n = np.asarray(worst["normal"], dtype=float)
    n /= (np.linalg.norm(n) + 1e-12)
    p_link = np.asarray(worst["p_link"], dtype=float)

    link_id = link_id_from_obj(worst["rob_obj"])
    Jv = jacobian_point_fn(link_id, q, p_link)  # 3×N

    # Damped least-squares: Δq = J^T (J J^T + λ² I)^-1 Δx
    Δx = n * gap  # desired translation of that surface point
    JJt = Jv @ Jv.T
    Δq = Jv.T @ np.linalg.solve(JJt + (lam**2) * np.eye(3), Δx)

    # Cap & scale
    cap = np.deg2rad(dq_deg_cap)
    Δq = np.clip(Δq, -cap, cap)
    q_new = clamp_to_limits(q + alpha * Δq)

    return q_new, {
        "action": "avoid_step",
        "gap": float(gap),
        "delta_x": Δx,
        "delta_q": Δq,
        "link_id": link_id,
        "type": worst["type"],
    }

# --------- Internals ----------

def _worst_clearance(robot_objs, scene_objs):
    """Smallest positive separation using pairwise fcl.distance (nearest points)."""
    dreq = fcl.DistanceRequest(enable_nearest_points=True)
    best = None
    best_d = np.inf

    # robot_objs/scene_objs can be either plain fcl objects or wrappers with .fcl_object
    def _obj(x): return x.fcl_object if hasattr(x, "fcl_object") else x

    for rob in robot_objs:
        ro = _obj(rob)
        for obs in scene_objs:
            so = _obj(obs)
            dres = fcl.DistanceResult()
            fcl.distance(ro, so, dreq, dres)
            d = dres.min_distance
            if d < best_d:
                p_link, p_obs = [np.array(p, dtype=float) for p in dres.nearest_points]
                n = p_link - p_obs
                nn = np.linalg.norm(n)
                if nn < 1e-12:
                    # Coincident points: pick an arbitrary, stable axis
                    n = np.array([1.0, 0.0, 0.0])
                else:
                    n /= nn
                best_d = d
                best = {
                    "clearance": float(d),   # positive
                    "p_link": p_link,
                    "p_obs": p_obs,
                    "normal": n,             # obstacle -> link
                    "rob_obj": ro,
                    "obs_obj": so,
                    "type": "clearance",
                }
    return best


def _worst_penetration(robot_objs, scene_objs):
    """
    Largest penetration among robot-vs-scene using collide() contacts.
    We scan pairs to keep it self-contained & explicit.
    """
    req = fcl.CollisionRequest(num_max_contacts=100000, enable_contact=True)

    def _obj(x): return x.fcl_object if hasattr(x, "fcl_object") else x

    worst = None
    max_pen = -1.0

    for rob in robot_objs:
        ro = _obj(rob)
        for obs in scene_objs:
            so = _obj(obs)
            res = fcl.CollisionResult()
            fcl.collide(ro, so, req, res)
            # res.contacts is a list of fcl.Contact
            for c in getattr(res, "contacts", []):
                pen = getattr(c, "penetration_depth", 0.0)
                if pen > max_pen:
                    n = np.array(c.normal, dtype=float)
                    n /= (np.linalg.norm(n) + 1e-12)
                    p = np.array(c.pos, dtype=float)
                    # Synthesize two points on each side of the contact plane
                    p_link = p + 0.5 * pen * n
                    p_obs  = p - 0.5 * pen * n
                    worst = {
                        "clearance": -float(pen),  # negative
                        "p_link": p_link,
                        "p_obs": p_obs,
                        "normal": n,               # obstacle -> link (approx)
                        "rob_obj": ro,
                        "obs_obj": so,
                        "type": "penetration",
                    }
                    max_pen = pen
    return worst
