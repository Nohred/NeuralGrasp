"""Calibración mano-ojo con cámara FIJA (eye-to-hand) usando ``cv2.calibrateHandEye``.

``cv2.calibrateHandEye`` está planteada para cámara EN la mano (eye-in-hand):
recibe ``gripper2base`` (= T_base_gripper) y ``target2cam`` (= T_cam_target) y
devuelve ``cam2gripper`` (= T_gripper_cam), usando que el tablero está fijo en la base:

    T_base_target = T_base_gripper_i @ X @ T_cam_target_i   (constante),  X = T_gripper_cam

Con cámara fija el tablero va en el gripper y lo constante es:

    T_gripper_target = inv(T_base_gripper_i) @ T_base_cam @ T_cam_target_i

que es la misma ecuación cambiando T_base_gripper por su INVERSA (T_gripper_base).
Por eso se le pasan las inversas como "gripper2base" y el resultado "cam2gripper"
se lee como ``T_base_cam``. Lo verifican tests/test_handeye.py (sin ruido, error ~0)
y un test negativo (sin invertir, el resultado es incorrecto).
"""
from __future__ import annotations

import cv2
import numpy as np

from .transforms import invert_T, make_T

METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def calibrate_handeye_raw(gripper2base_list, target2cam_list, method: str = "tsai") -> np.ndarray:
    """Envoltura directa de ``cv2.calibrateHandEye`` con matrices 4x4 (sin invertir nada).

    Devuelve "cam2gripper" como 4x4. Úsala solo si sabes qué representa cada entrada;
    para cámara fija usa :func:`calibrate_eye_to_hand`.
    """
    if method not in METHODS:
        raise ValueError(f"Método desconocido {method!r}; opciones: {sorted(METHODS)}")
    A = [np.asarray(T, dtype=np.float64) for T in gripper2base_list]
    B = [np.asarray(T, dtype=np.float64) for T in target2cam_list]
    if len(A) != len(B):
        raise ValueError("Las listas de poses del robot y de detecciones deben tener la misma longitud")
    if len(A) < 3:
        raise ValueError("Se necesitan al menos 3 poses (en la práctica, muchas más)")
    for T in A + B:
        if T.shape != (4, 4):
            raise ValueError("Cada pose debe ser una matriz 4x4")
    R, t = cv2.calibrateHandEye(
        [T[:3, :3] for T in A], [T[:3, 3].reshape(3, 1) for T in A],
        [T[:3, :3] for T in B], [T[:3, 3].reshape(3, 1) for T in B],
        method=METHODS[method],
    )
    return make_T(R, t.reshape(3))


def calibrate_eye_to_hand(T_base_gripper_list, T_cam_target_list, method: str = "tsai") -> np.ndarray:
    """Estima ``T_base_cam`` con cámara fija y tablero montado en el gripper.

    Args:
        T_base_gripper_list: poses del TCP en la base (4x4, mm), p. ej. de ``CRobT``.
        T_cam_target_list: poses del tablero en la cámara (4x4, mm), p. ej. de ChArUco.
        method: 'tsai', 'park', 'horaud', 'andreff' o 'daniilidis'.

    Returns:
        ``T_base_cam`` (4x4, mm).
    """
    T_gripper_base_list = [invert_T(T) for T in T_base_gripper_list]
    return calibrate_handeye_raw(T_gripper_base_list, T_cam_target_list, method)
