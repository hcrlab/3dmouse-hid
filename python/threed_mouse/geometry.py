from typing import Optional, Tuple

import numpy as np


def unpack_T(T) -> Tuple[np.ndarray, np.ndarray]:
    """ Returns the rotation matrix and translation separately

    Returns (R, p)
    """
    return T[..., :3, :3], T[..., :3, 3]


def unpack_R(R) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """ Returns the individual axes of the rotation matrix.
    """
    return R[...,:3, 0], R[...,:3, 1], R[...,:3, 2]


def pack_R(ax, ay, az, as_homogeneous=False):
    """ Returns a rotation matrix with the supplied axis columns.

    R = [ax, ay, az]
    """
    ax_v = np.atleast_2d(ax)
    ay_v = np.atleast_2d(ay)
    az_v = np.atleast_2d(az)
    assert ax_v.shape[0] == ay_v.shape[0] == az_v.shape[0]
    if as_homogeneous:
        R = np.empty((ax_v.shape[0], 4, 4))
        R[:] = np.eye(4)
    else:
        R = np.empty((ax_v.shape[0], 3, 3))
        R[:] = np.eye(3)
    R[...,:3, 0] = ax
    R[...,:3, 1] = ay
    R[...,:3, 2] = az
    return np.squeeze(R)


def pack_Rp(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """ Packs the provided rotation matrix (R) and position (p) into a homogeneous transform
    matrix.
    """
    # np.atleast_3d puts the extra dimension at the back but we need it at the front
    Rv = np.atleast_2d(R)
    Rb = Rv.view()
    if Rv.ndim == 2:
        Rb = Rv[None, :, :]
    # The user can pass in a single R for many P, or a single P for many R. We'll size the output for
    # the expected result of broadcasting.
    pb = np.atleast_2d(p)
    num_results = max(Rb.shape[0], pb.shape[0])
    T = np.tile(np.eye(4)[None,...], (num_results, 1,1))
    T[..., :3, :3] = Rb
    T[..., :3, 3] = pb
    if Rv.ndim == 2:
        return T.squeeze()
    else:
        return T

def R_to_angle(R: np.ndarray) -> np.ndarray:
    return np.arccos(np.clip((np.trace(R, axis1=-1, axis2=-2) - 1) / 2.,-1, 1))

def R_to_rot_vector(R: np.ndarray) -> np.ndarray:
    theta = R_to_angle(R)
    with np.errstate(invalid='ignore', divide='ignore'):
        # undefined if theta is 0 but we handle that in the following line
        aa = theta /(2 * np.sin(theta))*np.array([R[...,2,1]-R[...,1,2], R[...,0,2]-R[...,2,0], R[...,1,0]-R[...,0,1]])
    return np.where(~np.isnan(theta) & (theta != 0.0), aa, 0).T

def invert_T(T: np.ndarray):
    """ Inverts the provided transform matrix using the explicit formula leveraging the
    orthogonality of R and the sparsity of the transform.

    Specifically, denote T = h(R, t) where h(.,.) is a function mapping the rotation R and
    translation t to a homogeneous matrix defined by those parameters. Then

      inv(T) = inv(h(R,t)) = h(R', -R't).
    """
    R, t = unpack_T(T)
    R_trans = np.swapaxes(R, -1, -2)
    return pack_Rp(R_trans, np.squeeze(-R_trans @ t[..., None]))

class FrameVelocityEstimator:
    def __init__(self, dt):
        self.T_prev = None
        self.T_diff = None
        self.last_dt = None
        self.dt = dt

    @property
    def is_available(self):
        return self.T_diff is not None

    def update(self, T, dt=None):
        if self.T_prev is not None:
            self.T_diff = (invert_T(self.T_prev) @ T)
        self.T_prev = T
        self.last_dt = dt

    def get_twist(self, small_angle=False) -> Optional[np.ndarray]:
        if self.T_diff is None:
            return None
        dt = self.last_dt if self.last_dt is not None else self.dt
        diff = np.reshape(self.T_diff, (-1, 4,4))
        out = np.zeros((diff.shape[0], 6))
        out[:, :3] = self.T_diff[...,:3,3]
        if small_angle:
            # If the angle is small, the difference matrix is very close to I + an infinitesimal rotation.
            # This is good up to about theta=0.1 (alot!)
            out[:, 3] = self.T_diff[..., 2,1]
            out[:, 4] = self.T_diff[..., 0,2]
            out[:, 5] = self.T_diff[..., 1,0]
        else:
            out[:, 3:] = R_to_rot_vector(self.T_diff[...,:3, :3])
        return np.squeeze(out / dt)


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )


def so3_exp(omega: np.ndarray) -> np.ndarray:
    """Exponential map from so(3) to SO(3)."""
    w = np.asarray(omega, dtype=float).reshape(3)
    theta = float(np.linalg.norm(w))
    if theta < 1e-9:
        # First-order Taylor: I + K + 1/2 K^2
        K = skew(w)
        return np.eye(3, dtype=float) + K + 0.5 * (K @ K)
    axis = w / theta
    K = skew(axis)
    s = float(np.sin(theta))
    c = float(np.cos(theta))
    return np.eye(3, dtype=float) + s * K + (1.0 - c) * (K @ K)


def integrate_twist_stepwise(
    v_local: np.ndarray,
    w_local: np.ndarray,
    until_time: float,
    n_steps_per_sec: int,
) -> np.ndarray:
    """Integrate a body-frame twist into a sequence of local points."""
    step = 1.0 / float(n_steps_per_sec)
    n = 1 + int(until_time * n_steps_per_sec)
    result = np.empty((n, 3), dtype=float)
    result[0] = 0.0
    rot_step = so3_exp(np.asarray(w_local, dtype=float).reshape(3) * step)
    linear_step = np.asarray(v_local, dtype=float).reshape(3) * step
    for i in range(1, n):
        result[i] = (rot_step @ result[i - 1]) + linear_step
    return result


def apply_pose(local_points: np.ndarray, position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Transform local points into world frame."""
    pts = np.asarray(local_points, dtype=float)
    pos = np.asarray(position, dtype=float).reshape(3)
    rot = np.asarray(rotation, dtype=float).reshape(3, 3)
    return (rot @ pts.T).T + pos
