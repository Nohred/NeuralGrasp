"""Cliente de sockets contra un servidor falso que imita a comm.mod (sin RobotStudio)."""
import importlib.util
import os
import socket

import pytest

from robot.client.rapid_client import RobotVisionController
import numpy as np

from calibration.transforms import R_to_quat_rapid, euler_rapid_to_R
from tests.fake_rapid_server import P_HOME, FakeRapidServer, FakeRapidState, reply_for

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


@pytest.mark.parametrize("msg,expected", [
    ("HOME", "OK"),
    ("MOVE_POSE,1,2,3,4,5,6", "OK"),
    ("MOVE_POSE,1,2,3", "ERR_FORMATO"),
    ("MOVE_POSE,1,2,3,a,5,6", "ERR_FORMATO"),
    ("GRASP,1,2,3,0,45", "OK"),
    ("GRIPPER,1", "OK"),
    ("GRIPPER,", "ERR_FORMATO"),
    ("FOO,1", "ERR_CMD_NO_RECONOCIDO"),
    ("PING", "ERR_SINTAXIS"),
    ("GET_POSE", "POSE,281.85,5.73,310.95,0.044739,-0.811529,0.582552,-0.007207"),
])
def test_fake_server_reply_table(msg, expected):
    assert reply_for(msg) == expected


def test_get_pose_reply_fits_rapid_string():
    # Peor caso: coordenadas de 3 cifras negativas y componentes negativas del cuaternión.
    st = FakeRapidState(pose=(-475.55, -475.55, -475.55, -0.5, -0.5, -0.5, -0.5))
    assert len(st.handle("GET_POSE")) <= 80


def test_client_basic_commands():
    with FakeRapidServer() as srv:
        robot = RobotVisionController(port=srv.port, timeout=5.0)
        assert robot.connect()
        assert robot.go_home()
        assert robot.move_pose_euler(300, -20.5, 100, 0, 180, 0)
        assert robot.set_gripper(True)
        assert robot.set_gripper(False)
        assert robot.send_command("FOO,1") == "ERR_CMD_NO_RECONOCIDO"
        robot.disconnect()
    assert srv.received[:5] == [
        "HOME",
        "MOVE_POSE,300.00,-20.50,100.00,0.00,180.00,0.00",
        "GRIPPER,1",
        "GRIPPER,0",
        "FOO,1",
    ]


def test_get_pose_follows_moves():
    with FakeRapidServer() as srv:
        robot = RobotVisionController(port=srv.port, timeout=5.0)
        assert robot.connect()
        home = robot.get_pose()
        assert np.allclose(home[:3], P_HOME[:3]) and np.allclose(home[3:], P_HOME[3:], atol=1e-6)
        assert robot.move_pose_euler(300, -20, 150, 10, 170, -30)
        pose = robot.get_pose()
        robot.disconnect()
    assert np.allclose(pose[:3], [300, -20, 150])
    q_expected = R_to_quat_rapid(euler_rapid_to_R(10, 170, -30))
    assert np.allclose(pose[3:], q_expected, atol=1e-6)
    assert srv.too_long == []


def test_get_pose_returns_none_on_unexpected_reply(monkeypatch):
    robot = RobotVisionController()
    monkeypatch.setattr(robot, "send_command", lambda cmd: "ERR_SINTAXIS")
    assert robot.get_pose() is None
    monkeypatch.setattr(robot, "send_command", lambda cmd: "POSE,1,2,3")
    assert robot.get_pose() is None


def test_send_without_connection_returns_err():
    assert RobotVisionController().send_command("HOME") == "ERR"


def test_connect_fails_cleanly_when_nothing_listens():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()  # puerto libre sin nadie escuchando
    assert RobotVisionController(port=port, timeout=1.0).connect() is False


def _load_pick_place():
    path = os.path.join(REPO_ROOT, "experiments", "02_robot_pick_place", "run.py")
    spec = importlib.util.spec_from_file_location("pick_place_run", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_pick_and_place_sequence_messages(monkeypatch):
    mod = _load_pick_place()
    monkeypatch.setattr(mod.time, "sleep", lambda s: None)
    with FakeRapidServer() as srv:
        robot = RobotVisionController(port=srv.port, timeout=5.0)
        assert robot.connect()
        mod.ejecutar_pick_and_place(robot, "CUBO", [323.38, 58.88, 38.31], [-1.58, 274.70, 46.57], z_offset=80.0)
        robot.disconnect()
    assert srv.received == [
        "MOVE_POSE,323.38,58.88,118.31,0.00,180.00,0.00",
        "MOVE_POSE,323.38,58.88,38.31,0.00,180.00,0.00",
        "GRIPPER,1",
        "MOVE_POSE,323.38,58.88,118.31,0.00,180.00,0.00",
        "MOVE_POSE,-1.58,274.70,126.57,0.00,180.00,0.00",
        "MOVE_POSE,-1.58,274.70,46.57,0.00,180.00,0.00",
        "GRIPPER,0",
        "MOVE_POSE,-1.58,274.70,126.57,0.00,180.00,0.00",
    ]
    assert srv.too_long == []  # todos los mensajes caben en un string RAPID (80 caracteres)
