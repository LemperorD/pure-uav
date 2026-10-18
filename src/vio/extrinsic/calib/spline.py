"""Uniform cubic translation / cumulative SO(3) spline with analytic derivatives.

R_WC maps camera vectors into the fixed target/world frame. Body angular velocity
satisfies dR_WC/dt = R_WC [omega_C]x. No Euler-angle interpolation or numeric
trajectory differentiation is used in the estimator.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def skew(v):
    v = np.asarray(v)
    out = np.zeros(v.shape[:-1] + (3, 3))
    out[..., 0, 1], out[..., 0, 2] = -v[..., 2], v[..., 1]
    out[..., 1, 0], out[..., 1, 2] = v[..., 2], -v[..., 0]
    out[..., 2, 0], out[..., 2, 1] = -v[..., 1], v[..., 0]
    return out


def mv(m, v):
    return np.einsum("...ij,...j->...i", m, v)


def basis(u, h):
    u = np.asarray(u)
    b = np.stack(((1-u)**3, 3*u**3-6*u*u+4,
                  -3*u**3+3*u*u+3*u+1, u**3), axis=-1) / 6
    d = np.stack((-3*(1-u)**2, 9*u*u-12*u,
                  -9*u*u+6*u+3, 3*u*u), axis=-1) / (6*h)
    dd = np.stack((6*(1-u), 18*u-12, -18*u+6, 6*u), axis=-1) / (6*h*h)
    return b, d, dd


class PoseSpline:
    def __init__(self, start, spacing, n_controls):
        self.start, self.h, self.n = float(start), float(spacing), int(n_controls)

    def support(self, t):
        z = (np.asarray(t) - self.start) / self.h
        i = np.floor(z).astype(int)
        if np.any(i < 0) or np.any(i > self.n - 4):
            raise ValueError("时间落在样条有效区间外；请检查时间戳/偏移/窗口。")
        return i, z-i

    def evaluate(self, t, rotvecs, positions):
        i, u = self.support(t)
        ids = i[:, None] + np.arange(4)
        b, db, ddb = basis(u, self.h)
        p4 = np.asarray(positions)[ids]
        p = np.einsum("nk,nkj->nj", b, p4)
        a = np.einsum("nk,nkj->nj", ddb, p4)
        mats = Rotation.from_rotvec(rotvecs).as_matrix()
        r4 = mats[ids]
        rel = np.swapaxes(r4[:, :-1], -1, -2) @ r4[:, 1:]
        phi = Rotation.from_matrix(rel.reshape(-1, 3, 3)).as_rotvec().reshape(-1, 3, 3)
        beta = np.stack((1-b[:, 0], b[:, 2]+b[:, 3], b[:, 3]), axis=1)
        dbeta = np.stack((-db[:, 0], db[:, 2]+db[:, 3], db[:, 3]), axis=1)
        ddbeta = np.stack((-ddb[:, 0], ddb[:, 2]+ddb[:, 3], ddb[:, 3]), axis=1)
        aa = Rotation.from_rotvec((phi*beta[..., None]).reshape(-1, 3)).as_matrix().reshape(-1, 3, 3, 3)
        a1, a2, a3 = aa[:, 0], aa[:, 1], aa[:, 2]
        r = r4[:, 0] @ a1 @ a2 @ a3
        tail1 = np.swapaxes(a2 @ a3, -1, -2)
        tail2 = np.swapaxes(a3, -1, -2)
        v1 = mv(tail1, phi[:, 0]*dbeta[:, 0, None])
        v2 = mv(tail2, phi[:, 1]*dbeta[:, 1, None])
        v3 = phi[:, 2]*dbeta[:, 2, None]
        omega = v1 + v2 + v3
        alpha = (mv(tail1, phi[:, 0]*ddbeta[:, 0, None])
                 + mv(tail2, phi[:, 1]*ddbeta[:, 1, None])
                 + phi[:, 2]*ddbeta[:, 2, None]
                 - np.cross(v2+v3, v1) - np.cross(v3, v2))
        return r, p, a, omega, alpha

    def translation_design(self, t):
        i, u = self.support(t)
        b = basis(u, self.h)[0]
        out = np.zeros((len(t), self.n))
        out[np.arange(len(t))[:, None], i[:, None]+np.arange(4)] = b
        return out


def predict_imu(r_wc, acceleration_w, omega_c, alpha_c, r_ci, p_ci, gravity_w, bg, ba):
    """T_CI = (r_ci,p_ci) maps I->C; p_ci is IMU origin expressed in C.
    p_WI = p_WC + R_WC p_CI; f_I = R_WI^T (p_WI'' - g_W).
    The tangential AND centripetal lever-arm acceleration are essential.
    """
    r_cw = np.swapaxes(r_wc, -1, -2)
    lever = np.cross(alpha_c, p_ci) + np.cross(omega_c, np.cross(omega_c, p_ci))
    gyro = (omega_c @ r_ci) + bg
    accel = (mv(r_cw, acceleration_w-gravity_w) + lever) @ r_ci + ba
    return gyro, accel


def invert_transform(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out
