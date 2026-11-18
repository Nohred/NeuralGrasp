"""Graba poses del TCP (tVision, wobj0) con GET_POSE para usarlas con ``run.py --poses csv``.

Flujo pensado para poses puestas a mano (jog) en RobotStudio:
  1. comm.mod (con GET_POSE) en ejecución; ejecutar este script (se conecta y queda esperando).
  2. Llevar el robot a una pose (ver la nota sobre jog abajo) y pulsar Enter aquí: se lee
     GET_POSE y se agrega una fila al CSV. Escribir ``q`` + Enter para terminar.

NOTA (sin verificar): con el programa RAPID en ejecución no se puede hacer jog. Hay que
detenerlo (Stop), hacer jog y reanudarlo con Play SIN "PP to Main"; se espera que la
conexión TCP sobreviva a la pausa. Si se pierde, este script lo avisa y conserva lo grabado.
Alternativa automática sin jog: ``run.py --poses robotstudio``.

Ejemplo:
  python experiments/04_handeye_synthetic/record_poses.py --out experiments/04_handeye_synthetic/poses_jog.csv
"""
from __future__ import annotations

import argparse
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from robot.client import RobotVisionController  # noqa: E402

HEADER = "x,y,z,q1,q2,q3,q4"


def append_pose(path: str, pose) -> None:
    """Agrega una pose (x,y,z,q1..q4) al CSV, creando el encabezado si hace falta."""
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="\n", encoding="utf-8") as fh:
        if new:
            fh.write("# Poses grabadas con GET_POSE (tVision, wobj0). mm y cuaternion RAPID [w,x,y,z].\n")
            fh.write(HEADER + "\n")
        fh.write(",".join(f"{v:.6f}" for v in pose) + "\n")


def record(robot, path: str, input_fn=input, max_poses: int | None = None) -> int:
    """Bucle de grabación; devuelve cuántas poses se agregaron."""
    n = 0
    while max_poses is None or n < max_poses:
        cmd = input_fn(f"[{n} grabadas] Enter = grabar pose actual, q = terminar: ").strip().lower()
        if cmd == "q":
            break
        try:
            pose = robot.get_pose()
        except OSError as exc:
            print(f"[Error] Conexión perdida: {exc}. Se conservan {n} poses en {path}.")
            break
        if pose is None:
            continue
        append_pose(path, pose)
        n += 1
        print("    " + ", ".join(f"{v:.4f}" for v in pose))
    return n


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", required=True, help="CSV de salida (se agregan filas si ya existe).")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=55555)
    p.add_argument("--timeout", type=float, default=600.0,
                   help="Timeout del socket (s); largo porque el usuario hace jog entre lecturas.")
    args = p.parse_args(argv)

    robot = RobotVisionController(host=args.host, port=args.port, timeout=args.timeout)
    if not robot.connect():
        return 1
    try:
        n = record(robot, args.out)
    finally:
        robot.disconnect()
    print(f"{n} poses agregadas a {args.out}. Siguiente: "
          f"python experiments/04_handeye_synthetic/run.py --poses csv --csv {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
