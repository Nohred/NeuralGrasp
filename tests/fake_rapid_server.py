"""Servidor TCP falso que imita a robot/rapid/comm.mod (módulo ServidorVision).

Reproduce el formato de respuestas de comm.mod para probar el cliente sin RobotStudio:
  - Al aceptar: envía "Conectado a RAPID Vision".
  - "HOME" -> "OK".
  - Mensaje con coma: CMD = texto antes de la primera coma.
      MOVE_POSE (6 números), GRASP (5), GRIPPER (1) -> "OK"; si faltan números o
      alguno no es numérico -> "ERR_FORMATO" (ParseDatos ignora valores sobrantes).
      Otro CMD -> "ERR_CMD_NO_RECONOCIDO".
  - Sin coma y distinto de HOME -> "ERR_SINTAXIS".
No mueve nada: solo registra los mensajes recibidos en ``received``.
Limitación: RAPID recibe en un string de máx. 80 caracteres; aquí se registra
``too_long`` en lugar de simular el truncado.
"""
from __future__ import annotations

import socket
import threading

GREETING = "Conectado a RAPID Vision"
EXPECTED_ARGS = {"MOVE_POSE": 6, "GRASP": 5, "GRIPPER": 1}
RAPID_MAX_STR = 80


def _parse_ok(payload: str, n: int) -> bool:
    """Imita ParseDatos de comm.mod: toma los primeros n tokens y exige que sean números."""
    tokens = (payload + ",").split(",")
    if len(tokens) - 1 < n:
        return False
    try:
        [float(t) for t in tokens[:n]]
    except ValueError:
        return False
    return True


def reply_for(msg: str) -> str:
    if msg == "HOME":
        return "OK"
    if "," in msg:
        cmd, payload = msg.split(",", 1)
        if cmd in EXPECTED_ARGS:
            return "OK" if _parse_ok(payload, EXPECTED_ARGS[cmd]) else "ERR_FORMATO"
        return "ERR_CMD_NO_RECONOCIDO"
    return "ERR_SINTAXIS"


class FakeRapidServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen(1)
        self.host, self.port = self._sock.getsockname()
        self.received: list[str] = []
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
                if len(msg) > RAPID_MAX_STR:
                    self.too_long.append(msg)
                conn.sendall(reply_for(msg).encode())
