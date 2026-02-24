"""Renderer-agnostic twist preview utilities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from threed_mouse.geometry import apply_pose, integrate_twist_stepwise, so3_exp


def _as_vec3(value: np.ndarray | list[float] | tuple[float, float, float]) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(3)


def _as_twist6(value: np.ndarray | list[float] | tuple[float, float, float, float, float, float]) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(6)


def _as_pose44(value: np.ndarray) -> np.ndarray:
    return np.asarray(value, dtype=float).reshape(4, 4)


def _normalized(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return np.zeros(3, dtype=float)
    return np.asarray(v, dtype=float) / n


def compute_twist_alignment(
    v_goal_world: np.ndarray,
    w_goal_world: np.ndarray,
    twist_true: Optional[np.ndarray],
) -> float:
    """Return alignment score in [0, 1], where 1 is perfect agreement."""
    if twist_true is None:
        return 1.0
    v_true = np.asarray(twist_true[:3], dtype=float)
    w_true = np.asarray(twist_true[3:], dtype=float)

    v_dir = _normalized(v_goal_world)
    w_dir = _normalized(w_goal_world)
    v_true_dir = _normalized(v_true)
    w_true_dir = _normalized(w_true)

    disagreement = 0.0
    if not (np.allclose(v_dir, 0.0) or np.allclose(v_true_dir, 0.0)):
        v_agreement = float(np.dot(v_true_dir, v_dir))
        disagreement += 1.0 - abs(v_agreement)
    else:
        disagreement += float(np.linalg.norm(v_true) + np.linalg.norm(v_goal_world))

    if not (np.allclose(w_dir, 0.0) or np.allclose(w_true_dir, 0.0)):
        w_agreement = float(np.dot(w_true_dir, w_dir))
        disagreement += 1.0 - abs(w_agreement)
    else:
        disagreement += float(np.linalg.norm(w_true) + np.linalg.norm(w_goal_world))

    disagreement = float(np.clip(disagreement, 0.0, 1.0))
    return 1.0 - disagreement


@dataclass(frozen=True)
class TwistPreview:
    local_points: np.ndarray
    end_rotation: np.ndarray
    world_points: Optional[np.ndarray] = None
    alignment_score: Optional[float] = None


def compute_twist_preview(
    twist: np.ndarray | list[float] | tuple[float, float, float, float, float, float],
    *,
    horizon_s: float = 1.0,
    steps_per_sec: int = 10,
    pose: Optional[np.ndarray] = None,
    twist_true: Optional[np.ndarray] = None,
    v_goal_world: Optional[np.ndarray] = None,
    w_goal_world: Optional[np.ndarray] = None,
) -> TwistPreview:
    """Functional helper for twist preview integration."""
    twist_vec = _as_twist6(twist)
    v_local = twist_vec[:3]
    w_local = twist_vec[3:]
    local_pts = integrate_twist_stepwise(
        v_local,
        w_local,
        horizon_s,
        steps_per_sec,
    )
    end_rot = so3_exp(w_local * horizon_s)
    world_pts = None
    if pose is not None:
        pose_matrix = _as_pose44(pose)
        world_pts = apply_pose(local_pts, pose_matrix[:3, 3], pose_matrix[:3, :3])

    alignment = None
    if v_goal_world is not None and w_goal_world is not None:
        alignment = compute_twist_alignment(v_goal_world, w_goal_world, twist_true)

    return TwistPreview(
        local_points=local_pts,
        end_rotation=end_rot,
        world_points=world_pts,
        alignment_score=alignment,
    )
