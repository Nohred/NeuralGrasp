"""Calibración eye-to-hand: recuperación sin ruido, test negativo y métricas."""
import numpy as np
import pytest

from calibration import (
    METHODS,
    SyntheticScene,
    calibrate_eye_to_hand,
    calibrate_handeye_raw,
    consistency_residual,
    perturb_transform,
    pose_error,
    summarize,
)

# Sin ruido los 5 métodos recuperan T_base_cam con error ~1e-10 mm/° (float64).
# Tolerancia documentada en experiments/04_handeye_synthetic/README.md.
TOL = 1e-6


@pytest.fixture(scope="module")
def data():
    scene = SyntheticScene.from_config()
    rng = np.random.default_rng(2024)
    poses = scene.sample_gripper_poses(15, rng)
    return scene, poses, scene.detect(poses, rng)


# Requeridos: Tsai, Park, Daniilidis. Horaud y Andreff también pasan con la misma
# tolerancia (Andreff/Daniilidis quedan ~1e-11, los demás ~1e-13).
@pytest.mark.parametrize("method", list(METHODS))
def test_noise_free_recovery(data, method):
    scene, poses, det = data
    et, er = pose_error(calibrate_eye_to_hand(poses, det, method), scene.T_base_cam)
    assert et < TOL and er < TOL, f"{method}: {et:.2e} mm, {er:.2e} deg"


def test_not_inverting_poses_gives_wrong_result(data):
    """Pasar T_base_gripper directamente (como eye-in-hand) con cámara fija es INCORRECTO."""
    scene, poses, det = data
    T_wrong = calibrate_handeye_raw(poses, det, "tsai")
    et, er = pose_error(T_wrong, scene.T_base_cam)
    assert et > 100.0 or er > 10.0
    # ...mientras que con las inversas sí se recupera:
    T_ok = calibrate_handeye_raw([np.linalg.inv(T) for T in poses], det, "tsai")
    assert pose_error(T_ok, scene.T_base_cam)[0] < TOL


def test_consistency_residual_zero_at_truth_and_grows_with_error(data):
    scene, poses, det = data
    r0 = consistency_residual(poses, det, scene.T_base_cam)
    assert r0["trans_rms_mm"] < 1e-9 and r0["rot_rms_deg"] < 1e-9
    assert np.allclose(r0["T_gripper_target_mean"], scene.T_gripper_target, atol=1e-9)
    r1 = consistency_residual(poses, det, perturb_transform(scene.T_base_cam, 5.0, 1.0, [1, 1, 1]))
    assert r1["trans_rms_mm"] > 1.0 and r1["rot_rms_deg"] > 0.1


def test_noisy_calibration_is_reasonable():
    scene = SyntheticScene.from_config()
    rng = np.random.default_rng(5)
    poses = scene.sample_gripper_poses(20, rng)
    det = scene.detect(poses, rng, 0.5, 0.1)
    et, er = pose_error(calibrate_eye_to_hand(poses, det, "park"), scene.T_base_cam)
    assert et < 10.0 and er < 1.0


def test_input_validation(data):
    _, poses, det = data
    with pytest.raises(ValueError):
        calibrate_eye_to_hand(poses[:2], det[:2])
    with pytest.raises(ValueError):
        calibrate_eye_to_hand(poses, det[:-1])
    with pytest.raises(ValueError):
        calibrate_eye_to_hand(poses, det, "no_existe")


def test_pose_error_exact():
    T = np.eye(4)
    et, er = pose_error(perturb_transform(T, 3.0, 2.0, "y"), T)
    assert et == pytest.approx(3.0) and er == pytest.approx(2.0)


def test_summarize_ignores_nan():
    s = summarize([1.0, 2.0, 3.0, np.nan])
    assert s["n"] == 3 and s["mean"] == pytest.approx(2.0) and s["std"] == pytest.approx(1.0)
    assert summarize([np.nan])["n"] == 0
