"""Escena sintética: determinismo con semilla, filtro de visibilidad y ruido."""
import numpy as np
import pytest

from calibration import SceneConfig, SyntheticScene, make_T, workspace_box
from calibration.transforms import axis_angle_to_R, invert_T, rotation_angle_deg


@pytest.fixture
def scene():
    return SyntheticScene.from_config()


def test_camera_frame_is_opencv_convention(scene):
    T = scene.T_base_cam
    cam, look = np.array(scene.config.cam_position_mm), np.array(scene.config.cam_look_at_mm)
    z = (look - cam) / np.linalg.norm(look - cam)
    assert np.allclose(T[:3, 2], z)          # z hacia adelante
    assert T[2, 1] < 0                       # y de la imagen apunta hacia abajo (z base negativa)
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0)


def test_same_seed_same_poses_and_detections(scene):
    a = scene.sample_gripper_poses(10, np.random.default_rng(7))
    b = scene.sample_gripper_poses(10, np.random.default_rng(7))
    assert all(np.array_equal(x, y) for x, y in zip(a, b))
    da = scene.detect(a, np.random.default_rng(1), 1.0, 0.5)
    db = scene.detect(b, np.random.default_rng(1), 1.0, 0.5)
    assert all(np.array_equal(x, y) for x, y in zip(da, db))


def test_different_seed_different_poses(scene):
    a = scene.sample_gripper_poses(5, np.random.default_rng(1))
    b = scene.sample_gripper_poses(5, np.random.default_rng(2))
    assert not np.allclose(a[0], b[0])


def test_sampled_poses_satisfy_filters(scene, rng):
    c = scene.config
    ws = c.workspace
    for T in scene.sample_gripper_poses(40, rng):
        assert np.all(T[:3, 3] >= ws[:, 0]) and np.all(T[:3, 3] <= ws[:, 1])
        dist, ang, zc = scene.view_geometry(T)
        assert zc > 0
        assert ang <= c.max_view_angle_deg
        assert c.min_distance_mm <= dist <= c.max_distance_mm


def _pose_facing_camera(scene, position, extra_tilt_deg=0.0):
    """Herramienta en ``position`` apuntando exactamente a la cámara, inclinada ``extra_tilt_deg``."""
    d = scene.T_base_cam[:3, 3] - position
    d /= np.linalg.norm(d)
    x = np.cross([0, 0, 1.0], d)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, np.cross(d, x), d])
    return make_T(R @ axis_angle_to_R("x", extra_tilt_deg), position)


def test_visibility_angle_threshold(scene):
    # La pose del tablero está a 50 mm del TCP; con la herramienta apuntando a la cámara
    # el ángulo de vista es ~0. Inclinándola se cruza el umbral de 60°.
    p = np.array([300.0, 0.0, 300.0])
    assert scene.is_visible(_pose_facing_camera(scene, p, 0.0))
    _, ang_in, _ = scene.view_geometry(_pose_facing_camera(scene, p, 55.0))
    _, ang_out, _ = scene.view_geometry(_pose_facing_camera(scene, p, 65.0))
    assert ang_in < 60.0 < ang_out
    assert scene.is_visible(_pose_facing_camera(scene, p, 55.0))
    assert not scene.is_visible(_pose_facing_camera(scene, p, 65.0))


def test_visibility_rejects_board_facing_away(scene):
    p = np.array([300.0, 0.0, 300.0])
    assert not scene.is_visible(_pose_facing_camera(scene, p, 180.0))


def test_visibility_distance_limits(scene):
    cam = scene.T_base_cam[:3, 3]
    z = scene.T_base_cam[:3, 2]
    # Tablero (TCP + 50 mm hacia la cámara) a ~250 mm y a ~1100 mm de la cámara.
    near = _pose_facing_camera(scene, cam + z * 300.0)
    far = _pose_facing_camera(scene, cam + z * 1150.0)
    assert scene.view_geometry(near)[0] == pytest.approx(250.0, abs=1e-6)
    assert not scene.is_visible(near)
    assert not scene.is_visible(far)
    ok = _pose_facing_camera(scene, cam + z * 700.0)
    assert scene.is_visible(ok)


def test_detection_without_noise_is_exact_chain(scene, rng):
    poses = scene.sample_gripper_poses(5, rng)
    for T_bg, T_ct in zip(poses, scene.detect(poses, rng)):
        expected = invert_T(scene.T_base_cam) @ T_bg @ scene.T_gripper_target
        assert np.allclose(T_ct, expected)


def test_detection_noise_statistics(scene):
    rng = np.random.default_rng(0)
    T_bg = scene.sample_gripper_poses(1, rng)[0]
    truth = scene.T_cam_target_true(T_bg)
    det = scene.detect([T_bg] * 4000, rng, noise_trans_mm=2.0, noise_rot_deg=0.5)
    dt = np.array([d[:3, 3] - truth[:3, 3] for d in det])
    assert np.allclose(dt.mean(axis=0), 0.0, atol=0.15)
    assert np.allclose(dt.std(axis=0), 2.0, rtol=0.06)
    # |rotvec| con 3 componentes N(0, σ²) sigue una Maxwell: media = 2σ·sqrt(2/π).
    ang = np.array([rotation_angle_deg(d[:3, :3], truth[:3, :3]) for d in det])
    assert ang.mean() == pytest.approx(0.5 * 2 * np.sqrt(2 / np.pi), rel=0.05)


def test_translation_only_mode_has_fixed_rotation(scene, rng):
    poses = scene.sample_gripper_poses(8, rng, mode="translation_only")
    for T in poses[1:]:
        assert np.allclose(T[:3, :3], poses[0][:3, :3])
    assert len({tuple(np.round(T[:3, 3], 6)) for T in poses}) == 8


def test_workspace_box_scales_with_reach():
    small, large = workspace_box(475), workspace_box(580)
    assert np.all(np.abs(large) >= np.abs(small))
    assert SyntheticScene.from_config(SceneConfig(reach_mm=580)).config.workspace.shape == (3, 2)
