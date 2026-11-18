"""Servidor TCP falso que imita a robot/rapid/comm.mod (módulo ServidorVision).

Reproduce el formato de respuestas de comm.mod para probar el cliente sin RobotStudio:
  - Al aceptar: envía "Conectado a RAPID Vision".
  - "HOME" -> "OK" (la pose pasa a pHome).
  - "GET_POSE" -> "POSE,x,y,z,q1,q2,q3,q4" de la pose actual (mm, 2 dec.; cuaternión, 6 dec.).
  - Mensaje con coma: CMD = texto antes de la primera coma.
      MOVE_POSE (6 números), GRASP (5), GRIPPER (1) -> "OK"; si faltan números o
      alguno no es numérico -> "ERR_FORMATO" (ParseDatos ignora valores sobrantes).
      Otro CMD -> "ERR_CMD_NO_RECONOCIDO".
  - Sin coma y distinto de HOME/GET_POSE -> "ERR_SINTAXIS".

Simula el movimiento de forma ideal: MOVE_POSE/GRASP fijan la pose exacta pedida, con la
orientación de OrientZYX (sin cinemática, alcance ni errores de configuración).
Limitación: RAPID recibe en un string de máx. 80 caracteres; aquí se registra
``too_long`` en lugar de simular el truncado.
"""
from __future__ import annotations

import socket
import threading

import numpy as np

from calibration.transforms import R_to_quat_rapid, euler_rapid_to_R

GREETING = "Conectado a RAPID Vision"
EXPECTED_ARGS = {"MOVE_POSE": 6, "GRASP": 5, "GRIPPER": 1}
RAPID_MAX_STR = 80
# pHome de comm.mod.
P_HOME = (281.85, 5.73, 310.95, 0.044738964, -0.811528945, 0.58255236, -0.007207268)


def _parse(payload: str, n: int):
    """Imita ParseDatos de comm.mod: toma los primeros n tokens y exige que sean números."""
    tokens = (payload + ",").split(",")
    if len(tokens) - 1 < n:
        return None
    try:
        return [float(t) for t in tokens[:n]]
    except ValueError:
        return None


class FakeRapidState:
    """Lógica de AtenderVision con la pose del TCP como estado."""

    def __init__(self, pose=P_HOME):
        self.pose = tuple(pose)
        self.gripper_closed = False

    def handle(self, msg: str) -> str:
        if msg == "HOME":
            self.pose = P_HOME
            return "OK"
        if msg == "GET_POSE":
            x, y, z, *q = self.pose
            return "POSE," + ",".join([f"{v:.2f}" for v in (x, y, z)] + [f"{v:.6f}" for v in q])
        if "," in msg:
            cmd, payload = msg.split(",", 1)
            if cmd not in EXPECTED_ARGS:
                return "ERR_CMD_NO_RECONOCIDO"
            a = _parse(payload, EXPECTED_ARGS[cmd])
            if a is None:
                return "ERR_FORMATO"
            if cmd == "MOVE_POSE":
                q = R_to_quat_rapid(euler_rapid_to_R(a[3], a[4], a[5]))   # OrientZYX(rz, ry, rx)
                self.pose = (a[0], a[1], a[2], *q)
            elif cmd == "GRASP":
                q = R_to_quat_rapid(euler_rapid_to_R(180.0, 0.0, a[4]))   # OrientZYX(ang, 0, 180)
                self.pose = (a[0], a[1], a[2], *q)
            else:
                self.gripper_closed = a[0] == 1
            return "OK"
        return "ERR_SINTAXIS"


def reply_for(msg: str) -> str:
    """Respuesta de un servidor recién iniciado (en pHome) a un único mensaje."""
    return FakeRapidState().handle(msg)


class FakeRapidServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        self.host, self.port = self._sock.getsockname()
        self.state = FakeRapidState()
        self.received: list[str] = []
        self.sent: list[str] = []
        self.too_long: list[str] = []
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        try:
            self._sock.close()
        finally:
            self._thread.join(timeout=2)

    def _serve(self):
        try:
            conn, _ = self._sock.accept()
        except OSError:
            return
        with conn:
            conn.sendall(GREETING.encode())
            while True:
                try:
                    data = conn.recv(1024)
                except OSError:
                    return
                if not data:
                    return  # cliente cerró (en RAPID: ERR_SOCK_CLOSED -> vuelve a SocketAccept)
                msg = data.decode("ascii")
                self.received.append(msg)
                reply = self.state.handle(msg)
                for m in (msg, reply):
                    if len(m) > RAPID_MAX_STR:
                        self.too_long.append(m)
                self.sent.append(reply)
                conn.sendall(reply.encode())


def unit_quat_ok(pose, atol=1e-5) -> bool:
    return bool(np.isclose(np.linalg.norm(pose[3:]), 1.0, atol=atol))
