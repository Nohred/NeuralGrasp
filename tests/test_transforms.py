"""Conversiones de orientación, inversas y perturb_transform."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from calibration.transforms import (
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


def random_T(rng):
    return make_T(Rotation.random(random_state=rng).as_matrix(), rng.uniform(-500, 500, 3))


def Rx(d):
    c, s = np.cos(np.radians(d)), np.sin(np.radians(d))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(d):
    c, s = np.cos(np.radians(d)), np.sin(np.radians(d))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(d):
    c, s = np.cos(np.radians(d)), np.sin(np.radians(d))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# ------------------------------------------------------------- 4x4 básicos
def test_invert_T_roundtrip(rng):
    for _ in range(50):
        T = random_T(rng)
        assert np.allclose(T @ invert_T(T), np.eye(4), atol=1e-10)
        assert np.allclose(invert_T(invert_T(T)), T, atol=1e-10)
        assert is_rigid(T)


# ------------------------------------------------------------ cuaterniones
def test_quat_rapid_order_is_scalar_first():
    # [q1,q2,q3,q4] = [w,x,y,z]: [0,0,1,0] es 180° alrededor de y (herramienta boca abajo).
    assert np.allclose(quat_rapid_to_R([0, 0, 1, 0]), Ry(180), atol=1e-12)
    # 90° alrededor de z: w = cos(45°), z = sin(45°).
    h = np.sqrt(0.5)
    assert np.allclose(quat_rapid_to_R([h, 0, 0, h]), Rz(90), atol=1e-12)


def test_quat_rapid_roundtrip(rng):
    for _ in range(100):
        R = Rotation.random(random_state=rng).as_matrix()
        q = R_to_quat_rapid(R)
        assert q[0] >= 0 and np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(quat_rapid_to_R(q), R, atol=1e-12)


def test_pose_rapid_roundtrip_with_pHome():
    # pHome de comm.mod (cuaternión de 9 decimales, unitario).
    pose = [281.85, 5.73, 310.95, 0.044738964, -0.811528945, 0.58255236, -0.007207268]
    T = pose_rapid_to_T(*pose)
    assert is_rigid(T, atol=1e-6)
    back = T_to_pose_rapid(T)
    q = np.array(pose[3:])
    # q y -q son la misma rotación; R_to_quat_rapid devuelve w >= 0 (aquí w > 0 ya).
    assert np.allclose(back[:3], pose[:3])
    assert np.allclose(back[3:], q / np.linalg.norm(q), atol=1e-9)


# ------------------------------------------------------------- Euler RAPID
def test_euler_rapid_matches_orientzyx_definition(rng):
    # MOVE_POSE,x,y,z,rx,ry,rz -> OrientZYX(rz, ry, rx) = Rz(rz) Ry(ry) Rx(rx).
    for _ in range(50):
        rx, ry, rz = rng.uniform(-170, 170), rng.uniform(-80, 80), rng.uniform(-170, 170)
        assert np.allclose(euler_rapid_to_R(rx, ry, rz), Rz(rz) @ Ry(ry) @ Rx(rx), atol=1e-12)


def test_euler_rapid_pick_place_orientation():
    # La secuencia de pick & place usa rx=0, ry=180, rz=0 -> igual al cuaternión [0,0,1,0].
    assert np.allclose(euler_rapid_to_R(0, 180, 0), quat_rapid_to_R([0, 0, 1, 0]), atol=1e-12)


def test_euler_rapid_roundtrip(rng):
    for _ in range(100):
        e = np.array([rng.uniform(-179, 179), rng.uniform(-85, 85), rng.uniform(-179, 179)])
        assert np.allclose(R_to_euler_rapid(euler_rapid_to_R(*e)), e, atol=1e-9)


# ----------------------------------------------------------- ángulo geodésico
def test_rotation_angle_small_and_large():
    assert rotation_angle_deg(np.eye(3), axis_angle_to_R("x", 1e-6)) == pytest.approx(1e-6, rel=1e-6)
    assert rotation_angle_deg(axis_angle_to_R([1, 1, 0], 135.0)) == pytest.approx(135.0)
    assert rotation_angle_deg(axis_angle_to_R("z", 180.0), np.eye(3)) == pytest.approx(180.0)


# ---------------------------------------------------------- perturb_transform
@pytest.mark.parametrize("frame", ["local", "parent"])
@pytest.mark.parametrize("axis", ["x", "y", "z", [1.0, -2.0, 0.5]])
@pytest.mark.parametrize("trans_mm,rot_deg", [(0.0, 0.0), (1.0, 0.0), (0.0, 0.5), (5.0, 2.0), (12.3, 45.0)])
def test_perturb_transform_exact_magnitude(rng, frame, axis, trans_mm, rot_deg):
    T = random_T(rng)
    Tp = perturb_transform(T, trans_mm, rot_deg, axis, frame=frame)
    assert is_rigid(Tp)
    assert np.linalg.norm(Tp[:3, 3] - T[:3, 3]) == pytest.approx(trans_mm, abs=1e-9)
    assert rotation_angle_deg(T[:3, :3], Tp[:3, :3]) == pytest.approx(rot_deg, abs=1e-9)


def test_perturb_transform_direction_local_vs_parent():
    T = make_T(Rz(90), [100, 0, 0])
    # 'local': x local de T es +y del padre.
    assert np.allclose(perturb_transform(T, 10, 0, "x", "local")[:3, 3], [100, 10, 0])
    assert np.allclose(perturb_transform(T, 10, 0, "x", "parent")[:3, 3], [110, 0, 0])


def test_perturb_transform_bad_frame():
    with pytest.raises(ValueError):
        perturb_transform(np.eye(4), 1, 1, "x", frame="world")
