"""Transformaciones homogéneas 4x4 y conversiones de orientación.

Convenciones (ver docs/coordinate_frames.md):
  - Unidades: milímetros y grados en la interfaz pública; radianes solo internamente.
  - ``T_a_b`` es la pose del marco ``b`` expresada en ``a``: ``p_a = T_a_b @ p_b``.
  - Cuaternión RAPID: ``[q1, q2, q3, q4] = [w, x, y, z]`` (escalar primero).
    scipy usa ``[x, y, z, w]`` (escalar al final); aquí se convierte explícitamente.
  - ``OrientZYX(rz, ry, rx)`` de RAPID = rotación intrínseca Z, luego Y', luego X'':
    ``R = Rz(rz) @ Ry(ry) @ Rx(rx)``. Es lo que hace ``MOVE_POSE,x,y,z,rx,ry,rz``
    en robot/rapid/comm.mod (línea 59: ``OrientZYX(args{6}, args{5}, args{4})``).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

_AXES = {"x": np.array([1.0, 0.0, 0.0]), "y": np.array([0.0, 1.0, 0.0]), "z": np.array([0.0, 0.0, 1.0])}


# ---------------------------------------------------------------- básicos 4x4
def make_T(R=None, t=None) -> np.ndarray:
    """Construye una matriz homogénea 4x4 a partir de R (3x3) y t (3,) en mm."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float)
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert_T(T) -> np.ndarray:
    """Inversa cerrada de una transformación rígida: [R^T, -R^T t]."""
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def is_rigid(T, atol: float = 1e-9) -> bool:
    """True si T es una transformación rígida válida (R ortonormal, det = +1)."""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    return (
        T.shape == (4, 4)
        and np.allclose(T[3], [0, 0, 0, 1], atol=atol)
        and np.allclose(R.T @ R, np.eye(3), atol=atol)
        and np.isclose(np.linalg.det(R), 1.0, atol=atol)
    )


# ------------------------------------------------------ cuaterniones (RAPID)
def quat_rapid_to_R(q) -> np.ndarray:
    """Cuaternión RAPID ``[q1, q2, q3, q4] = [w, x, y, z]`` -> matriz de rotación 3x3."""
    w, x, y, z = np.asarray(q, dtype=float)
    return Rotation.from_quat([x, y, z, w]).as_matrix()  # scipy: [x, y, z, w]


def R_to_quat_rapid(R) -> np.ndarray:
    """Matriz 3x3 -> cuaternión RAPID ``[w, x, y, z]`` normalizado, con ``w >= 0``."""
    x, y, z, w = Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()
    q = np.array([w, x, y, z])
    return -q if q[0] < 0 else q


def pose_rapid_to_T(x, y, z, q1, q2, q3, q4) -> np.ndarray:
    """``robtarget.trans`` (mm) + ``robtarget.rot`` (orden RAPID) -> T 4x4."""
    return make_T(quat_rapid_to_R([q1, q2, q3, q4]), [x, y, z])


def T_to_pose_rapid(T) -> np.ndarray:
    """T 4x4 -> ``[x, y, z, q1, q2, q3, q4]`` (mm, orden RAPID)."""
    T = np.asarray(T, dtype=float)
    return np.concatenate([T[:3, 3], R_to_quat_rapid(T[:3, :3])])


# ------------------------------------------------- Euler (MOVE_POSE / OrientZYX)
def euler_rapid_to_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """Ángulos de ``MOVE_POSE`` (grados) -> R, igual que ``OrientZYX(rz, ry, rx)``.

    ``R = Rz(rz) @ Ry(ry) @ Rx(rx)`` (secuencia intrínseca ZYX; en scipy, ``'ZYX'``).
    """
    return Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True).as_matrix()


def R_to_euler_rapid(R) -> np.ndarray:
    """Inversa de :func:`euler_rapid_to_R`: devuelve ``(rx, ry, rz)`` en grados.

    Equivale a ``EulerZYX(\\X/\\Y/\\Z, rot)`` de RAPID. Cerca de ``ry = ±90°`` hay
    bloqueo de cardán y la descomposición no es única (scipy emite un aviso).
    """
    rz, ry, rx = Rotation.from_matrix(np.asarray(R, dtype=float)).as_euler("ZYX", degrees=True)
    return np.array([rx, ry, rz])


# ------------------------------------------------------------ ángulos y ejes
def axis_angle_to_R(axis, angle_deg: float) -> np.ndarray:
    """Rotación de ``angle_deg`` grados alrededor de ``axis`` ('x','y','z' o vector 3D)."""
    a = _unit_axis(axis)
    return Rotation.from_rotvec(np.deg2rad(angle_deg) * a).as_matrix()


def rotation_angle_deg(R1, R2=None) -> float:
    """Ángulo geodésico (grados) entre dos rotaciones: ``|log(R1^T R2)|``.

    Si ``R2`` es None, devuelve el ángulo de ``R1`` respecto a la identidad.
    Usa scipy (vía cuaternión), que es preciso también para ángulos muy pequeños,
    a diferencia de ``arccos((tr - 1)/2)``.
    """
    R1 = np.asarray(R1, dtype=float)
    Rrel = R1 if R2 is None else R1.T @ np.asarray(R2, dtype=float)
    return float(np.rad2deg(Rotation.from_matrix(Rrel).magnitude()))


def perturb_transform(T, trans_mm: float, rot_deg: float, axis="z", frame: str = "local") -> np.ndarray:
    """Perturba ``T`` con un error de magnitud EXACTA: ``trans_mm`` y ``rot_deg``.

    Se traslada ``trans_mm`` a lo largo de ``axis`` y se rota ``rot_deg`` alrededor
    del mismo ``axis``, manteniendo el origen del marco como centro de rotación.

    - ``frame='local'``: ``axis`` en el marco propio de T (``T' = T @ dT``).
    - ``frame='parent'``: ``axis`` en el marco padre de T; la rotación es alrededor
      del origen de T (no del origen del padre), para que la traslación sea exacta.

    Garantía: ``||t' - t|| == |trans_mm|`` y ``rotation_angle_deg(R, R') == |rot_deg|``
    (para ``|rot_deg| <= 180``). Pensado para estudiar después cómo un error de
    calibración T_base_cam afecta el agarre.
    """
    T = np.asarray(T, dtype=float)
    a = _unit_axis(axis)
    dR = axis_angle_to_R(a, rot_deg)
    R, t = T[:3, :3], T[:3, 3]
    if frame == "local":
        return make_T(R @ dR, t + R @ (trans_mm * a))
    if frame == "parent":
        return make_T(dR @ R, t + trans_mm * a)
    raise ValueError("frame debe ser 'local' o 'parent'")


def random_rotation_small(rng: np.random.Generator, sigma_deg: float) -> np.ndarray:
    """Rotación aleatoria con rotvec ~ N(0, sigma_deg^2 I) (grados por componente)."""
    rv = rng.normal(0.0, sigma_deg, size=3)
    return Rotation.from_rotvec(np.deg2rad(rv)).as_matrix()


def _unit_axis(axis) -> np.ndarray:
    if isinstance(axis, str):
        try:
            return _AXES[axis.lower()].copy()
        except KeyError as exc:
            raise ValueError(f"Eje desconocido: {axis!r}") from exc
    a = np.asarray(axis, dtype=float).reshape(3)
    n = np.linalg.norm(a)
    if n == 0:
        raise ValueError("El eje no puede ser el vector cero")
    return a / n
