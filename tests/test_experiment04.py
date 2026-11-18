"""Prueba de humo del experimento 04 (pocas repeticiones, sin gráficas)."""
import csv
import importlib.util
import os

import numpy as np
import pytest

from robot.client import RobotVisionController
from tests.fake_rapid_server import FakeRapidServer

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
EXP = os.path.join(REPO_ROOT, "experiments", "04_handeye_synthetic")


@pytest.fixture(scope="module")
def run_mod():
    spec = importlib.util.spec_from_file_location("exp04_run", os.path.join(EXP, "run.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_load_example_csv(run_mod):
    poses = run_mod.load_poses_csv(os.path.join(EXP, "poses_example.csv"))
    assert len(poses) == 20
    assert all(np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6) for T in poses)


def test_csv_rejects_non_unit_quaternion(run_mod, tmp_path):
    p = tmp_path / "bad.csv"
    p.write_text("x,y,z,q1,q2,q3,q4\n1,2,3,1,1,0,0\n", encoding="utf-8")
    with pytest.raises(ValueError):
        run_mod.load_poses_csv(str(p))


@pytest.mark.parametrize("poses_args", [[], ["--poses", "csv", "--csv", os.path.join(EXP, "poses_example.csv")]])
def test_smoke_run(run_mod, tmp_path, poses_args):
    out = tmp_path / "res"
    code = run_mod.main(poses_args + ["--n-reps", "2", "--pose-counts", "5,10", "--noise-trans-mm", "0,1",
                                      "--noise-rot-deg", "0,0.2", "--no-plots", "--out", str(out)])
    assert code == 0  # la prueba sin ruido pasó
    with open(out / "summary.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert {r["method"] for r in rows} == {"tsai", "park", "horaud", "andreff", "daniilidis"}


def test_robotstudio_dry_run_prints_commands(run_mod, tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        run_mod.main(["--poses", "robotstudio", "--rs-dry-run", "--rs-n-poses", "5", "--out", str(tmp_path)])
    assert exc.value.code == 0
    out = capsys.readouterr().out
    assert out.count("MOVE_POSE,") == 5
    assert all(len(line.strip()) <= 80 for line in out.splitlines() if "MOVE_POSE," in line)


def test_robotstudio_mode_against_fake_server(run_mod, tmp_path, capsys):
    """Ciclo completo: MOVE_POSE a poses sintéticas -> GET_POSE -> calibración.

    El servidor falso aplica OrientZYX igual que comm.mod, así que la diferencia
    pedida vs. leída solo viene del redondeo (2 decimales en ángulos/mm, 6 en q).
    """
    out = tmp_path / "res"
    with FakeRapidServer() as srv:
        code = run_mod.main(["--poses", "robotstudio", "--port", str(srv.port), "--rs-n-poses", "12",
                             "--n-poses", "10", "--n-reps", "2", "--pose-counts", "5,10",
                             "--noise-trans-mm", "0,1", "--noise-rot-deg", "0,0.2", "--no-plots",
                             "--out", str(out)])
    assert code == 0
    text = capsys.readouterr().out
    assert "CONSISTENTE" in text
    assert srv.received.count("GET_POSE") == 13          # 1 inicial + 1 por pose
    assert srv.too_long == []
    poses = run_mod.load_poses_csv(str(out / "robotstudio_poses.csv"))
    assert len(poses) == 12


def test_record_poses_against_fake_server(run_mod, tmp_path):
    spec = importlib.util.spec_from_file_location("exp04_rec", os.path.join(EXP, "record_poses.py"))
    rec = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rec)
    csv_path = str(tmp_path / "jog.csv")
    answers = iter(["", "", "q"])
    with FakeRapidServer() as srv:
        robot = RobotVisionController(port=srv.port, timeout=5.0)
        assert robot.connect()
        n = rec.record(robot, csv_path, input_fn=lambda prompt: next(answers))
        robot.disconnect()
    assert n == 2
    poses = run_mod.load_poses_csv(csv_path)
    assert len(poses) == 2 and np.allclose(poses[0][:3, 3], [281.85, 5.73, 310.95])
