import numpy as np

# Corrected Pose class
class Pose:
    def rmat_to_quat(self, rmat):
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

    def quat_to_rmat(self, quat):
        qx, qy, qz, qw = quat
        qx2, qy2, qz2 = qx*qx, qy*qy, qz*qz
        return [
            [1 - 2*qy2 - 2*qz2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx2 - 2*qz2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx2 - 2*qy2]
        ]

    def quat_dot(self, quat1, quat2):
        q1 = np.array(quat1)
        q2 = np.array(quat2)
        return float(np.dot(q1, q2))

    def quat_slerp(self, quat1, quat2, t):
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

    def quat_mul(self, quat1, quat2):
        x1, y1, z1, w1 = quat1
        x2, y2, z2, w2 = quat2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return [x, y, z, w]

    def abc_to_rmat(self, abc):
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

    def rmat_to_abc(self, rmat):
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
            x = np.sqrt(max(rmat[0,0]+1, 0))/2
            y = np.sqrt(max(rmat[1,1]+1, 0))/2
            z = np.sqrt(max(rmat[2,2]+1, 0))/2
            if rmat[1,0]<0: y = -y
            if rmat[2,0]<0: z = -z
        return [x*np.degrees(theta), y*np.degrees(theta), z*np.degrees(theta)]

    def T_to_xyzabc(self, T):
        T = np.array(T, dtype=float)
        abc = self.rmat_to_abc(T[:3,:3])
        return [T[0,3], T[1,3], T[2,3]] + abc

    def xyzabc_to_T(self, xyzabc):
        T = np.eye(4)
        T[:3,:3] = self.abc_to_rmat(xyzabc[3:])
        T[0,3], T[1,3], T[2,3] = xyzabc[0], xyzabc[1], xyzabc[2]
        return T.tolist()

    def transform_pose(self, xyzabc, from_frame=[0,0,0,0,0,0], to_frame=[0,0,0,0,0,0]):
        T_pose = np.array(self.xyzabc_to_T(xyzabc))
        T_from = np.array(self.xyzabc_to_T(from_frame))
        T_to   = np.array(self.xyzabc_to_T(to_frame))
        R_to, t_to = T_to[:3,:3], T_to[:3,3]
        R_to_inv = R_to.T
        t_to_inv = -R_to_inv @ t_to
        T_to_inv = np.eye(4)
        T_to_inv[:3,:3], T_to_inv[:3,3] = R_to_inv, t_to_inv
        T_result = T_to_inv @ T_from @ T_pose
        return self.T_to_xyzabc(T_result)

    def rotate_abc(self, abc, axis=[0,0,1], angle=0, local=False):
        T = np.array(self.abc_to_rmat(abc))
        axis = np.array(axis, dtype=float)
        if local:
            axis = T @ axis
        axis /= np.linalg.norm(axis)
        R = np.array(self.abc_to_rmat(list(axis*angle)))
        RT = R @ T
        return self.rmat_to_abc(RT)


    def align_abc(self, abc, align="Z", axis=[0, 0, 1]):
        """
        Rotate the pose so that its local X/Y/Z axis aligns with `axis`.
        - abc:    [ax, ay, az] axis-angle in degrees
        - align:  'X', 'Y', or 'Z'
        - axis:   target direction in world frame
        Returns a new [ax, ay, az] so that the requested local axis now points along `axis`.
        """
        # 1) Build current rotation matrix
        current_rmat = np.array(self.abc_to_rmat(abc))

        # 2) Pick the local axis column
        idx = {"x": 0, "y": 1, "z": 2}.get(align.lower())
        if idx is None:
            raise ValueError("align must be 'X', 'Y', or 'Z'")
        a = current_rmat[:, idx]
        a = a / np.linalg.norm(a)

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
        return self.rotate_abc(abc, orth.tolist(), rot_deg, local=False)
