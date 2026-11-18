"""Métricas de calibración mano-ojo (mm y grados)."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .transforms import invert_T, make_T, rotation_angle_deg


def pose_error(T_est, T_true):
    """Error contra verdad de terreno: ``(error_traslacion_mm, error_rotacion_deg)``.

    Traslación: ``||t_est - t_true||``. Rotación: ángulo geodésico entre R_est y R_true.
    """
    T_est = np.asarray(T_est, dtype=float)
    T_true = np.asarray(T_true, dtype=float)
    return (float(np.linalg.norm(T_est[:3, 3] - T_true[:3, 3])),
            rotation_angle_deg(T_est[:3, :3], T_true[:3, :3]))


def consistency_residual(T_base_gripper_list, T_cam_target_list, T_base_cam_est) -> dict:
    """Residuo de consistencia SIN verdad de terreno (sirve con datos reales).

    Para cada pose i se calcula ``T_gripper_target_i = inv(T_base_gripper_i) @ T_base_cam
    @ T_cam_target_i``. Si la calibración y los datos fueran perfectos, sería la misma
    matriz para todas las poses (el tablero está rígidamente montado en el gripper).
    Se reporta su dispersión respecto a la pose media.

    Returns:
        dict con ``trans_rms_mm``, ``trans_max_mm``, ``rot_rms_deg``, ``rot_max_deg``,
        ``per_pose_trans_mm``, ``per_pose_rot_deg`` y ``T_gripper_target_mean`` (4x4).
    """
    Ts = [invert_T(Tbg) @ np.asarray(T_base_cam_est) @ np.asarray(Tct)
          for Tbg, Tct in zip(T_base_gripper_list, T_cam_target_list)]
    t = np.array([T[:3, 3] for T in Ts])
    t_mean = t.mean(axis=0)
    R_mean = Rotation.from_matrix(np.array([T[:3, :3] for T in Ts])).mean().as_matrix()
    dt = np.linalg.norm(t - t_mean, axis=1)
    dr = np.array([rotation_angle_deg(T[:3, :3], R_mean) for T in Ts])
    return {
        "trans_rms_mm": float(np.sqrt(np.mean(dt ** 2))),
        "trans_max_mm": float(dt.max()),
        "rot_rms_deg": float(np.sqrt(np.mean(dr ** 2))),
        "rot_max_deg": float(dr.max()),
        "per_pose_trans_mm": dt,
        "per_pose_rot_deg": dr,
        "T_gripper_target_mean": make_T(R_mean, t_mean),
    }


def summarize(values) -> dict:
    """Media, desviación estándar y percentil 95 ignorando NaN; incluye ``n`` válidos."""
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {"mean": np.nan, "std": np.nan, "p95": np.nan, "n": 0}
    return {"mean": float(v.mean()), "std": float(v.std(ddof=1)) if v.size > 1 else 0.0,
            "p95": float(np.percentile(v, 95)), "n": int(v.size)}
