import numpy as np
from dorna2 import Dorna


def bisect_right(a, x):
    """
    Return the index i where to insert item x in list a to keep it sorted.
    Equivalent to Python's bisect.bisect_right.
    """
    lo, hi = 0, len(a)
    while lo < hi:
        mid = (lo + hi) // 2
        if x < a[mid]:
            hi = mid
        else:
            lo = mid + 1
    return lo


class path:
    def __init__(self, points, offsets=None):
        """
        points: list of points (list/array-like of equal-length vectors)
        offsets: optional list of {"t": float in [0,1], "offset": list/array same dim as points}
                 You can pass 'deviation' instead of 'offset' in each item; it’ll be accepted.
        """
        self.points = [np.asarray(p, dtype=float) for p in points]
        self.num_steps = len(self.points)
        if self.num_steps < 2:
            raise ValueError("Need at least two points")

        self.vel = self.velocities()

        # Initialize offsets/deviations
        self._dim = self.points[0].shape[0]
        self.offsets = self._normalize_offsets(offsets or [])

    # ----- geometry core -----
    def velocities(self):
        pts = np.asarray(self.points, dtype=float)
        n = len(pts)
        v = [None] * n
        v[0] = pts[1] - pts[0]
        v[-1] = pts[-1] - pts[-2]
        for i in range(1, n - 1):
            v[i] = 0.5 * (pts[i + 1] - pts[i - 1])
        return [np.asarray(vi) for vi in v]

    def get_point(self, t):
        """Linear interpolation across key points (t in [0,1])."""
        t = float(np.clip(t, 0.0, 1.0))
        n = t * (self.num_steps - 1)
        if np.isclose(n, np.round(n)):
            return self.points[int(np.round(n))].copy()
        n1 = int(np.floor(n))
        n2 = n1 + 1
        return self.mix_points(self.points[n1], self.points[n2], n - n1)

    def get_vel(self, t):
        """Linear interpolation across key points (t in [0,1])."""
        t = float(np.clip(t, 0.0, 1.0))
        n = t * (self.num_steps - 1)
        if np.isclose(n, np.round(n)):
            return self.points[int(np.round(n))].copy()
        n1 = int(np.floor(n))
        n2 = n1 + 1
        return self.mix_points(self.vel[n1], self.vel[n2], n - n1)

    def get_point_d(self, t):
        """get_point(t) + get_deviation(t)."""
        return self.get_point(t) + self.get_deviation(t)

    @staticmethod
    def mix_points(p1, p2, t):
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        return p1 + (p2 - p1) * t

    # ----- offsets / deviation -----
    def _normalize_offsets(self, items):
        """
        Accepts items with key 'offset' or 'deviation'. Ensures:
          - vectors are numpy arrays matching dimension
          - list sorted by t
          - (0, 0) and (1, 0) present, overriding any nonzero ends
        """
        norm = []
        for it in items:
            if "offset" in it:
                vec = it["offset"]
            elif "deviation" in it:
                vec = it["deviation"]
            else:
                raise ValueError("Each offset item must have 'offset' or 'deviation' key")
            t = float(it["t"])
            if not (0.0 <= t <= 1.0):
                raise ValueError("offset t must be in [0,1]")
            vec = np.asarray(vec, dtype=float)
            if vec.shape != (self._dim,):
                raise ValueError(f"offset vector must have dim {self._dim}, got {vec.shape}")
            norm.append({"t": t, "offset": vec})

        # Sort by t
        norm.sort(key=lambda x: x["t"])

        # Force zero at ends
        zero = np.zeros(self._dim, dtype=float)
        if not norm or norm[0]["t"] > 0.0 or not np.allclose(norm[0]["offset"], 0):
            # Insert or replace start
            if norm and np.isclose(norm[0]["t"], 0.0):
                norm[0] = {"t": 0.0, "offset": zero}
            else:
                norm.insert(0, {"t": 0.0, "offset": zero})
        if norm[-1]["t"] < 1.0 or not np.allclose(norm[-1]["offset"], 0):
            # Append or replace end
            if np.isclose(norm[-1]["t"], 1.0):
                norm[-1] = {"t": 1.0, "offset": zero}
            else:
                norm.append({"t": 1.0, "offset": zero})

        # Deduplicate same t (keep the last one before endpoints logic)
        dedup = []
        for item in norm:
            if dedup and np.isclose(dedup[-1]["t"], item["t"]):
                dedup[-1] = item
            else:
                dedup.append(item)

        # Extract arrays for fast search
        self._offset_ts = np.array([x["t"] for x in dedup], dtype=float)
        self._offset_vecs = [x["offset"] for x in dedup]
        return dedup

    def set_offsets(self, items):
        """Replace the offsets/deviations list."""
        self.offsets = self._normalize_offsets(items)

    def add_offset(self, t, vec=None, *, deviation=None):
        """Convenience to insert one offset/deviation at t."""
        v = vec if vec is not None else deviation
        if v is None:
            raise ValueError("Provide vec=... or deviation=...")
        items = list(self.offsets)
        items.append({"t": t, "offset": v})
        self.set_offsets(items)

    def get_deviation(self, t):
        """
        Smoothly interpolate offsets between enclosing keys using a smoothstep:
            s = 3u^2 - 2u^3, u in [0,1]
        Returns a numpy vector of same dim as points.
        """
        t = float(np.clip(t, 0.0, 1.0))

        ts = self._offset_ts
        vecs = self._offset_vecs

        # Fast paths for ends
        if t <= ts[0] + 1e-12:
            return vecs[0].copy()
        if t >= ts[-1] - 1e-12:
            return vecs[-1].copy()

        # Find right index (first ts[idx] > t), then segment is [idx-1, idx]
        idx = bisect_right(ts, t)
        t0, t1 = ts[idx - 1], ts[idx]
        v0, v1 = vecs[idx - 1], vecs[idx]

        # Normalized param in segment
        u = (t - t0) / max(t1 - t0, 1e-12)
        # Smoothstep blend for a gentle transition with zero slope at segment ends
        s = u * u * (3.0 - 2.0 * u)
        return v0 * (1.0 - s) + v1 * s


class path_gen:

	def __init__(self, motion_, j1_, j2_, steps_, kinematic_, tcp_):
		self.kin = kinematic_
		
		if steps_<3:
			print("number of steps is smaller that 3")
			return

		if motion_!="lmove" and motion_!="jmove":
			print("motion type should be either lmove or jmove")
			return

		self.steps = steps_
		self.motion = motion_
		
		self.singular = False

		self.velocity_treshold = 100

		if self.motion=="jmove":
			self.path = self.gen_jmove(j1_,j2_,steps_)

		if self.motion=="lmove":
			self.path = self.gen_lmove(j1_,j2_,steps_,tcp_)

	def gen_jmove(self, j1, j2, steps):
		j1 = np.array(j1)
		j2 = np.array(j2)

		points = []

		for i in range(steps):
			t = float(i)/float(steps-1)
			points.append((j1+(j2-j1)*t).tolist())

		return path(points)

	def gen_lmove(self, j1, j2, steps, tcp):
		self.kin.set_frame_tcp(frame=None, tcp=self.kin.xyzabc_to_mat(tcp))
		fw1 = np.array(self.kin.fw(j1))
		fw2 = np.array(self.kin.fw(j2))
		jj1 = np.array(j1)
		jj2 = np.array(j2)

		points = [np.array(j1).tolist()]
		last_joint = j1

		for i in range(1, steps):
			t = float(i)/float(steps-1)

			fw = fw1 + (fw2-fw1) * t
			jj = jj1 + (jj2-jj1) * t
			ikj = self.kin.ik_xyzj345([fw[0],fw[1],fw[2]] , [float(jj[3]),float(jj[4]),float(jj[5])], tcp, last_joint)

			new_joint = jj
			new_joint[:3]= ikj[:3]
			velocity = np.linalg.norm(np.array(new_joint) - np.array(last_joint))
			if velocity > self.velocity_treshold : 
				self.singular = True

			last_joint = new_joint
			points.append(last_joint)

		return path(points)