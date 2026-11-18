"""Cliente TCP para el servidor RAPID ``ServidorVision`` (robot/rapid/comm.mod).

Protocolo (ver docs/coordinate_frames.md y robot/rapid/README.md):
  - Al conectar, el servidor manda un saludo que se lee con ``recv``.
  - Request/response síncrono: un ``sendall`` -> un ``recv``. Texto ASCII sin delimitador.
  - Comandos: ``HOME``, ``MOVE_POSE,x,y,z,rx,ry,rz``, ``GRIPPER,1|0`` (y ``GRASP``, sin método aquí).
  - Respuestas: ``OK``, ``ERR_FORMATO``, ``ERR_CMD_NO_RECONOCIDO``, ``ERR_SINTAXIS``.

Este archivo contiene exactamente la clase que antes vivía en ``robot_comms.py`` (raíz);
la secuencia de pick & place se movió a ``experiments/02_robot_pick_place/run.py``.
"""
import socket


class RobotVisionController:
    def __init__(self, host="127.0.0.1", port=55555, timeout=15.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
            saludo = self.sock.recv(1024).decode()
            print(f"[Robot] Conectado: {saludo}")
            return True
        except Exception as e:
            print(f"[Error] No se pudo conectar: {e}")
            return False

    def send_command(self, cmd: str) -> str:
        if not self.sock: return "ERR"
        self.sock.sendall(cmd.encode('utf-8'))
        return self.sock.recv(1024).decode('utf-8')

    def go_home(self) -> bool:
        print("Moviendo a Home...")
        return self.send_command("HOME") == "OK"

    def move_pose_euler(self, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> bool:
        cmd = f"MOVE_POSE,{x:.2f},{y:.2f},{z:.2f},{rx:.2f},{ry:.2f},{rz:.2f}"
        return self.send_command(cmd) == "OK"

    def set_gripper(self, close_gripper: bool) -> bool:
        estado = 1 if close_gripper else 0
        cmd = f"GRIPPER,{estado}"
        return self.send_command(cmd) == "OK"

    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
            print("[Robot] Desconectado.")
