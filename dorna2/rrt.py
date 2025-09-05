
import time
import math
import sys
import numpy as np
from dorna2 import Dorna
from dorna2.simulation import check_collision, create_cube, create_sphere



import pybullet as p
import pybullet_data 


#Initiate pybullet if it's loaded
def pybullet_test():
    if 'p' in globals() and 'pybullet' in sys.modules and globals()['p'] is sys.modules['pybullet']:
        return True
    return False


if pybullet_test():
    # PyBullet setup 
    p.connect(p.GUI) 
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
    p.setGravity(0,0,-9.81)



def clamp(q, qmin, qmax):
    return np.minimum(np.maximum(q, qmin), qmax)

def wrap_angle(q):
    """If you want to wrap revolute joints to [-pi, pi], do it here.
    For now, noop to avoid surprising the user.
    """
    return q

class WeightedMetric:
    def __init__(self, wdiag: np.ndarray):
        self.w = np.asarray(wdiag, dtype=float)
        assert self.w.shape == (6,), "metric_weights must be (6,)"
        self.L = np.sqrt(self.w)            # L diag s.t. W = L^2
        self.Linv = 1.0 / self.L

    def dist(self, q1, q2):
        d = (q1 - q2) * self.L
        return float(np.linalg.norm(d))

    def steer(self, q_from, q_to, eta):
        """Steer from q_from towards q_to by length eta under the weighted metric.
        If ||q_to - q_from||_W <= eta, return q_to.
        """
        d = (q_to - q_from)
        d_scaled = d * self.L
        n = np.linalg.norm(d_scaled)
        if n <= 1e-12:
            return q_to.copy()
        if n <= eta:
            return q_to.copy()
        step_scaled = (eta / n) * d_scaled
        step = step_scaled * self.Linv
        return q_from + step

# -----------------------
# Constraint infrastructure
# -----------------------

def _constraint_dim(q0):
    try:
        h = constraint_h(q0)
        return int(h.size)
    except Exception:
        return 0

class ConstraintProjector:
    def __init__(self, qmin, qmax, tol=1e-5, max_iters=20, max_step=0.2):
        self.qmin = qmin
        self.qmax = qmax
        self.tol = tol
        self.max_iters = max_iters
        self.max_step = max_step  # limit on ||dq||_2 per projection step

    def has_constraints(self, q0):
        return _constraint_dim(q0) > 0

    def project(self, q: np.ndarray):
        """Project q onto the constraint manifold h(q)=0 using least-squares Newton.
        Returns (q_proj, success_bool).
        If no constraint functions, returns (q, True).
        """
        try:
            h = constraint_h(q)
        except Exception:
            return q.copy(), True  # no constraints provided
        if h.size == 0:
            return q.copy(), True

        qk = q.copy()
        for _ in range(self.max_iters):
            h = constraint_h(qk)
            if np.linalg.norm(h) <= self.tol:
                return clamp(qk, self.qmin, self.qmax), True
            Jh = constraint_jacobian(qk)  # (m,6)
            # Least-squares solve: minimize ||Jh * dq + h||
            # dq = -Jh^+ h
            try:
                dq = -np.linalg.pinv(Jh, rcond=1e-6).dot(h)
            except np.linalg.LinAlgError:
                return q, False
            # limit step
            n = np.linalg.norm(dq)
            if n > self.max_step:
                dq *= (self.max_step / max(n, 1e-12))
            qk = clamp(qk + dq, self.qmin, self.qmax)
        # final check
        ok = np.linalg.norm(constraint_h(qk)) <= 10 * self.tol
        return clamp(qk, self.qmin, self.qmax), bool(ok)

# ------------------------
# Edge checking (with projection when needed)
# ------------------------

def edge_collision_free(q0, q1, step_rad, projector: ConstraintProjector, metric: WeightedMetric):
    """Collision check from q0 to q1. If constraints exist, we interpolate with small
    steps and project each sample before calling is_state_collision().
    Otherwise we call the robot's fast segment checker if available.
    """
    # Detect if constraints exist by checking projector on q0
    has_c = projector.has_constraints(q0)

    if not has_c:
        # Prefer robot's fast segment checker if available
        try:
            ok = bool(is_edge_collision_free(q0, q1))  # type: ignore[name-defined]
            return ok
        except Exception:
            pass

    # Fallback / constrained mode: own sub-sampling + project + state collision
    # Compute number of sub-steps under weighted metric length
    Lw = max(1, int(math.ceil(metric.dist(q0, q1) / max(step_rad, 1e-6))))
    for i in range(1, Lw + 1):
        alpha = i / Lw
        q = (1 - alpha) * q0 + alpha * q1
        q, ok = projector.project(q)
        if not ok:
            return False
        if is_state_collision(q):  # type: ignore[name-defined]
            return False
    return True

# -----------------
# RRT-Connect trees
# -----------------

class Tree:
    def __init__(self, root: np.ndarray):
        self.nodes = [root.copy()]
        self.parents = [-1]

    def add(self, q: np.ndarray, parent_idx: int):
        self.nodes.append(q.copy())
        self.parents.append(parent_idx)
        return len(self.nodes) - 1

    def nearest(self, q: np.ndarray, metric: WeightedMetric):
        pts = np.asarray(self.nodes)
        # Linear scan is OK for thousands of nodes
        # Use weighted metric
        d = ((pts - q) * metric.L)  # scale
        idx = int(np.argmin(np.einsum('ij,ij->i', d, d)))
        return idx, self.nodes[idx]

    def path_to_root(self, idx: int):
        path = []
        while idx != -1:
            path.append(self.nodes[idx])
            idx = self.parents[idx]
        path.reverse()
        return [p.copy() for p in path]

# -----------------
# Samplers
# -----------------

def sample_uniform(qmin, qmax):
    return qmin + np.random.rand(6) * (qmax - qmin)

class InformedSampler:
    """Prolate hyperspheroid sampling in weighted space once a solution exists.
    We sample in the space scaled by metric.L where the metric becomes Euclidean.
    """
    def __init__(self, q_start, q_goal, metric: WeightedMetric):
        self.qs = q_start
        self.qg = q_goal
        self.metric = metric
        self.qs_s = self.qs * metric.L
        self.qg_s = self.qg * metric.L
        self.c_min = np.linalg.norm(self.qg_s - self.qs_s)
        # Orthonormal basis aligning foci axis
        e1 = (self.qg_s - self.qs_s)
        n = np.linalg.norm(e1)
        if n < 1e-12:
            e1 = np.array([1,0,0,0,0,0], dtype=float)
            n = 1.0
        e1 = e1 / n
        Q = np.eye(6)
        Q[:,0] = e1
        # Gram-Schmidt for remaining columns
        for k in range(1, 6):
            v = Q[:,k]
            for j in range(k):
                v = v - np.dot(v, Q[:,j]) * Q[:,j]
            nv = np.linalg.norm(v)
            if nv < 1e-9:
                # pick random orthogonal
                v = np.random.randn(6)
                for j in range(k):
                    v = v - np.dot(v, Q[:,j]) * Q[:,j]
                v /= np.linalg.norm(v)
            else:
                v /= nv
            Q[:,k] = v
        self.Q = Q

    def sample(self, c_best):
        if not np.isfinite(c_best) or c_best < self.c_min:
            # fallback uniform in box
            qmin, qmax = joint_limits()  # type: ignore[name-defined]
            return sample_uniform(qmin, qmax)
        # Radii along axes in scaled space
        a1 = c_best / 2.0
        a2 = math.sqrt(c_best**2 - self.c_min**2) / 2.0
        A = np.diag([a1, a2, a2, a2, a2, a2])
        # Sample unit ball
        u = np.random.randn(6)
        u /= max(np.linalg.norm(u), 1e-12)
        r = np.random.rand() ** (1/6)  # radius distribution for uniform ball
        x_unit = r * u
        x = self.Q @ (A @ x_unit)
        center_s = 0.5 * (self.qs_s + self.qg_s)
        q_s = center_s + x
        # Map back to joint space
        q = q_s * self.metric.Linv
        qmin, qmax = joint_limits()  # type: ignore[name-defined]
        return clamp(q, qmin, qmax)

# -----------------
# Planner
# -----------------

class RRTConnectPlanner:
    def __init__(self,
                 step_size=0.05,
                 goal_bias=0.1,
                 connect_iters=32,
                 max_samples=20000,
                 shortcut_attempts=300,
                 smooth_iterations=2,
                 seed=None):
        self.step_size = float(step_size)
        self.goal_bias = float(goal_bias)
        self.connect_iters = int(connect_iters)
        self.max_samples = int(max_samples)
        self.shortcut_attempts = int(shortcut_attempts)
        self.smooth_iterations = int(smooth_iterations)
        if seed is not None:
            np.random.seed(int(seed))

    def _sample(self, qmin, qmax, sampler, c_best, goal):
        if np.random.rand() < self.goal_bias:
            return goal.copy()
        if sampler is None:
            return sample_uniform(qmin, qmax)
        return sampler.sample(c_best)

    def plan(self, q_start: np.ndarray, q_goal: np.ndarray):
        qmin, qmax = joint_limits()  # type: ignore[name-defined]
        W = metric_weights()         # type: ignore[name-defined]
        metric = WeightedMetric(W)
        projector = ConstraintProjector(qmin, qmax)

        # Project start/goal (if constraints exist)
        q_s, ok_s = projector.project(clamp(q_start, qmin, qmax))
        q_g, ok_g = projector.project(clamp(q_goal,  qmin, qmax))
        if not (ok_s and ok_g):
            return {"success": False, "reason": "Cannot project start/goal onto constraints."}
        if is_state_collision(q_s) or is_state_collision(q_g):  # type: ignore[name-defined]
            return {"success": False, "reason": "Start or goal in collision."}

        T_a = Tree(q_s)
        T_b = Tree(q_g)
        best_path = None
        best_cost = np.inf
        sampler = None
        informed_ready = False
        n_samples = 0

        # If informed sampling is desired, initialize sampler
        try:
            sampler = InformedSampler(q_s, q_g, metric)
            informed_ready = False
        except Exception:
            sampler = None
            informed_ready = False

        for it in range(self.max_samples):
            print("sample: ",it)
            n_samples += 1
            # Alternate trees: grow A toward sample, then connect B toward new, then swap
            for swap in (False, True):
                Ta, Tb = (T_a, T_b) if not swap else (T_b, T_a)
                q_rand = self._sample(qmin, qmax, sampler if (informed_ready) else None,
                                      best_cost, Tb.nodes[0])

                idx_near, q_near = Ta.nearest(q_rand, metric)
                # Steer once toward sample
                q_new = metric.steer(q_near, q_rand, self.step_size)
                q_new, okp = projector.project(q_new)
                if not okp:
                    continue
                if not edge_collision_free(q_near, q_new, self.step_size, projector, metric):
                    continue
                idx_new = Ta.add(q_new, idx_near)

                # Try to connect other tree aggressively
                q_connect = q_new.copy()
                success_connect = False
                for _k in range(self.connect_iters):
                    idx2, qnear2 = Tb.nearest(q_connect, metric)
                    qnext = metric.steer(qnear2, q_connect, self.step_size)
                    qnext, okp2 = projector.project(qnext)
                    if not okp2:
                        break
                    if not edge_collision_free(qnear2, qnext, self.step_size, projector, metric):
                        break
                    idx2_new = Tb.add(qnext, idx2)
                    if metric.dist(qnext, q_connect) < 1e-6:
                        success_connect = True
                        break
                    # continue extending towards q_connect
                    
                if success_connect:
                    # Build path: Ta(root->q_new) + Tb(root->qnext) reversed if needed
                    if not swap:
                        path_a = Ta.path_to_root(idx_new)
                        path_b = Tb.path_to_root(idx2_new)
                        path_b.reverse()
                        path = path_a + path_b
                    else:
                        path_a = Tb.path_to_root(idx2_new)
                        path_b = Ta.path_to_root(idx_new)
                        path_b.reverse()
                        path = path_a + path_b
                    path = _dedup(path)
                    cost = path_length(path, metric)
                    if cost < best_cost:
                        best_cost = cost
                        best_path = path
                        informed_ready = True
            # (optional) early exit on good-enough
            if best_path is not None and best_cost < 1.5 * metric.dist(q_s, q_g):
                break

        if best_path is None:
            return {"success": False, "reason": "No path found."}

        # Smoothing: shortcutting
        best_path = shortcut_path(best_path, projector, metric, attempts=self.shortcut_attempts,
                                  step_rad=self.step_size)
        # Optional gentle smoothing (Chaikin)
        if self.smooth_iterations > 0:
            best_path = chaikin_smooth(best_path, iterations=self.smooth_iterations,
                                       projector=projector)
        best_path = _dedup(best_path)

        # Retiming
        try:
            vmax = velocity_limits()     # type: ignore[name-defined]
            amax = acceleration_limits() # type: ignore[name-defined]
        except Exception:
            vmax = np.full(6, 1.0)
            amax = np.full(6, 2.0)
        t_stamps = retime_trapezoid(best_path, vmax, amax)

        return {
            "success": True,
            "path": np.array(best_path),
            "time": np.array(t_stamps),
            "cost": float(best_cost),
            "nodes_a": len(T_a.nodes),
            "nodes_b": len(T_b.nodes),
            "samples": n_samples,
        }

# ---------------------
# Helpers: path metrics
# ---------------------

def path_length(path, metric: WeightedMetric):
    L = 0.0
    for i in range(len(path) - 1):
        L += metric.dist(path[i+1], path[i])
    return L

# ---------------------
# Smoothing / shortcut
# ---------------------

def _dedup(path):
    out = [path[0]]
    for p in path[1:]:
        if np.linalg.norm(p - out[-1]) > 1e-9:
            out.append(p)
    return out

def shortcut_path(path, projector: ConstraintProjector, metric: WeightedMetric,
                  attempts=300, step_rad=0.05):
    if len(path) < 3:
        return path
    path = [p.copy() for p in path]
    n = len(path)
    for _ in range(attempts):
        i = np.random.randint(0, n-2)
        j = np.random.randint(i+2, n)
        q0 = path[i]
        q1 = path[j]
        if edge_collision_free(q0, q1, step_rad, projector, metric):
            # Replace segment i..j with direct edge
            new_path = path[:i+1] + [q1] + path[j+1:]
            path = new_path
            n = len(path)
            if n < 3:
                break
    return path

# ---------------------
# Gentle smoothing (Chaikin) + resample
# ---------------------

def chaikin_once(points):
    pts = np.asarray(points)
    new_pts = [pts[0]]
    for i in range(len(pts)-1):
        p = pts[i]
        q = pts[i+1]
        Q = 0.75*p + 0.25*q
        R = 0.25*p + 0.75*q
        new_pts.extend([Q, R])
    new_pts.append(pts[-1])
    return np.array(new_pts)

def chaikin_smooth(path, iterations=2, projector: ConstraintProjector=None):
    pts = np.array(path)
    for _ in range(max(0, iterations)):
        pts = chaikin_once(pts)
        if projector is not None and projector.has_constraints(pts[0]):
            # Project each point to avoid drift
            proj = []
            for q in pts:
                q2, ok = projector.project(q)
                if not ok:
                    q2 = q
                proj.append(q2)
            pts = np.array(proj)
    # Downsample to reduce points
    pts = resample_by_step(pts, step=0.03)
    return [p.copy() for p in pts]

def resample_by_step(points, step=0.03):
    pts = np.asarray(points)
    out = [pts[0]]
    acc = 0.0
    for i in range(1, len(pts)):
        d = np.linalg.norm(pts[i] - out[-1])
        if d >= step:
            out.append(pts[i])
    if not np.allclose(out[-1], pts[-1]):
        out.append(pts[-1])
    return np.array(out)

# ---------------------
# Retiming: per-segment trapezoid (vel/acc)
# ---------------------

def segment_time_trapezoid(dq: np.ndarray, vmax: np.ndarray, amax: np.ndarray) -> float:
    """Minimal time to move dq under |dqdot|<=vmax, |ddq|<=amax with a symmetric trapezoid.
    If distance too short for full speed, triangular profile is used.
    
    Returns the time for the *slowest* joint (others can be executed within it).
    """
    dq_abs = np.abs(dq)
    T_j = np.zeros_like(dq_abs)
    for i in range(dq_abs.size):
        v = max(vmax[i], 1e-9)
        a = max(amax[i], 1e-9)
        d = dq_abs[i]
        d_star = v*v / a
        if d <= d_star:
            # triangular
            T = 2.0 * math.sqrt(d / a)
        else:
            T = (d - d_star) / v + 2.0 * v / a
        T_j[i] = T
    return float(np.max(T_j))

def retime_trapezoid(path, vmax, amax):
    pts = np.asarray(path)
    t = [0.0]
    for i in range(len(pts)-1):
        dt = segment_time_trapezoid(pts[i+1] - pts[i], vmax, amax)
        t.append(t[-1] + dt)
    return np.array(t)

# ---------------------
# Public convenience API
# ---------------------

def plan_rrt_connect(q_start, q_goal,
                     step_size=5,
                     goal_bias=0.1,
                     connect_iters=132,
                     max_samples=4000,
                     shortcut_attempts=300,
                     smooth_iterations=2,
                     seed=2):
    planner = RRTConnectPlanner(step_size=step_size,
                                goal_bias=goal_bias,
                                connect_iters=connect_iters,
                                max_samples=max_samples,
                                shortcut_attempts=shortcut_attempts,
                                smooth_iterations=smooth_iterations,
                                seed=seed)
    return planner.plan(np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float))

# ---------------------
# Minimal usage example (you will implement robot-side functions)
# ---------------------
def rrt(q0,q1):



    q0 = np.array(q0)#[0, -1.0, 0.5, 0.0, 0.7, 0.0])
    q1 = np.array(q1)#[1.2, -0.8, -0.4, 0.6, 0.3, -0.2])

    result = plan_rrt_connect(q0, q1, seed=42)
    print("Success:", result.get("success"))
    if result.get("success"):
        print("Total time:", result["time"][-1])
        print("Nodes A/B:", result["nodes_a"], result["nodes_b"])
        print(result)

    return result


if __name__ == "__main__":

    tool=[0,0,0,0,0,0]
    base_in_world=[0,0,0,0,0,0]
    frame_in_world=[0,0,0,0,0,0] 
    aux_dir=[[0, 0, 0], [0, 0, 0]]

    robot = Dorna()
    # Example stubs (replace with your implementations)


    #create objects
    scene = [] 
    for i in range(4):
        t = i * np.pi / 2 + np.pi/4
        l = 0.38
        tf = [l*np.cos(t),l*np.sin(t),0,0,0,0]
        scene.append(create_cube(tf, [0.05,0.05,0.8]))

    scene.append(create_cube([0,0,-0.6,0,0,0], [5,5,1]))

    load = [create_sphere([0,0,0,0,0,0], [0.02,0.02,0.3])]



    def joint_limits():
        return np.array([-180,-75,-156,-161,-180,-175]), np.array([160,200,156,180,160,175])

    def fk(q):
        p = robot.kinematic.fw(q) 
        m = robot.kinematic.xyzabc_to_mat(p)

        return np.array([p[0],p[1],p[2]]), m[:3,:3]

    def vee(S):
        return np.array([S[2,1], S[0,2], S[1,0]])

    def compute_pdots_Rdots(q, eps=1e-1):
        q = np.asarray(q, dtype=float).copy()
        p0, R0 = fk(q)
        assert p0.shape == (3,)
        assert R0.shape == (3,3)

        pdots = np.zeros((6, 3), dtype=float)
        Rdots = np.zeros((6, 3, 3), dtype=float)

        for i in range(6):
            qp = q.copy(); qp[i] += eps
            qm = q.copy(); qm[i] -= eps

            pp, Rp = fk(qp)
            pm, Rm = fk(qm)

            # Linear part: central difference
            pdots[i] = (pp - pm) / (2.0 * eps)

            # Rotation part: central difference, then project to tangent so (Rdot R^T) is skew
            Rdot_raw = (Rp - Rm) / (2.0 * eps)

            # Tangent projection: Ω = skew( Rdot_raw * R0^T ), then Rdot = Ω * R0
            Omega = Rdot_raw @ R0.T
            Omega = 0.5 * (Omega - Omega.T)        # force skew-symmetric
            Rdot = Omega @ R0

            Rdots[i] = Rdot

        return pdots, Rdots

    def jacobian(q):
        p,R = fk(q)

        pdots,Rdots = compute_pdots_Rdots(q)

        assert R.shape == (3,3)
        assert p.shape == (3,)
        assert Rdots.shape == (6,3,3)
        assert pdots.shape == (6,3)

        # Angular columns from Rdot_i:  ω_i = vee( Rdot_i * R^T )
        Jw = np.empty((3,6))
        for i in range(6):
            S = Rdots[i] @ R.T
            S = 0.5 * (S - S.T)   # enforce skew numerically
            Jw[:, i] = vee(S)

        Jv = pdots.T  # (3,6)

        # Stack to 6x6
        J = np.vstack([Jv, Jw])
        return J

    def is_state_collision(q):
        res = check_collision(q, tool=tool, load=load, scene=scene, base_in_world=base_in_world, frame_in_world=frame_in_world, aux_dir=aux_dir)
        if len(res["col"]) > 0:
            print("col")
            return True
        return False

    def is_edge_collision_free(q0, q1):
        for i in range(10):
            q = np.array(q0) + (np.array(q1)-np.array(q0)) * float(i)/9.0
            res = is_state_collision(q)
            if res:
                return False

        return True

    def metric_weights():
        return np.array([3,2,1.5,1,1,1], dtype=float)

    def constraint_h(q):
        return np.zeros(0)

    def constraint_jacobian(q):
        return np.zeros((0,6))
    
    def velocity_limits():
        return np.array([1.5,1.5,1.5,2.0,2.0,2.0])

    def acceleration_limits():
        return np.array([3.0,3.0,3.0,4.0,4.0,4.0])


    #rrt([0,0,0,0,0,0] , [-179,0,0,0,0,0])


    rrt([0,0,0,0,0,0],[10,0,0,0,0,0])