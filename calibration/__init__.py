"""Librería de calibración mano-ojo (eye-to-hand) sin hardware.

Todo en mm, grados y matrices homogéneas 4x4 (``T_a_b`` = pose de b en a).
La interfaz recibe listas de arrays 4x4 para poder enchufar después detecciones
reales de ChArUco y poses reales del robot sin reescribir nada.
"""
from .config import SceneConfig, workspace_box
from .handeye import METHODS, calibrate_eye_to_hand, calibrate_handeye_raw
from .metrics import consistency_residual, pose_error, summarize
from .synthetic import SyntheticScene, board_on_tool, look_at
from .transforms import (
    R_to_euler_rapid,
    R_to_quat_rapid,
    T_to_pose_rapid,
    axis_angle_to_R,
    euler_rapid_to_R,
    invert_T,
    is_rigid,
    make_T,
    perturb_transform,
    pose_rapid_to_T,
    quat_rapid_to_R,
    rotation_angle_deg,
)

__all__ = [
    "SceneConfig", "workspace_box", "METHODS", "calibrate_eye_to_hand", "calibrate_handeye_raw",
    "consistency_residual", "pose_error", "summarize", "SyntheticScene", "board_on_tool", "look_at",
    "R_to_euler_rapid", "R_to_quat_rapid", "T_to_pose_rapid", "axis_angle_to_R", "euler_rapid_to_R",
    "invert_T", "is_rigid", "make_T", "perturb_transform", "pose_rapid_to_T", "quat_rapid_to_R",
    "rotation_angle_deg",
]
