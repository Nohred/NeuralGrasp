"""Prueba de humo del experimento 04 (pocas repeticiones, sin gráficas)."""
import csv
import importlib.util
import os

import numpy as np
import pytest

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


def test_robotstudio_mode_is_documented_stub(run_mod, tmp_path, capsys):
    with pytest.raises(SystemExit) as exc:
        run_mod.main(["--poses", "robotstudio", "--out", str(tmp_path)])
    assert exc.value.code == 2
    assert "GET_POSE" in capsys.readouterr().out
