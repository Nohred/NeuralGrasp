"""Experimento 02: pick & place de cubo y cilindro en RobotStudio.

Mismo comportamiento que el antiguo ``robot_comms.py`` de la raíz: la secuencia y las
coordenadas son idénticas; solo el cliente se importa ahora desde ``robot/client``.
Requiere robot/rapid/comm.mod cargado y en ejecución en RobotStudio.
"""
import os
import sys
import time

# Permite ejecutar este script desde cualquier carpeta: añade la raíz del repo al path.
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from robot.client.rapid_client import RobotVisionController  # noqa: E402


# --- Función auxiliar para abstraer el ciclo Pick and Place ---
def ejecutar_pick_and_place(robot, nombre_obj, pick_coords, place_coords, z_offset=100.0):
    # Asumimos orientacion vertical boca abajo (RotY = 180)
    rx, ry, rz = 0.0, 180.0, 0.0
    px, py, pz = pick_coords
    dx, dy, dz = place_coords

    print(f"\n--- Secuencia para: {nombre_obj} ---")

    print("1. Acercandose al objeto (Approach)...")
    robot.move_pose_euler(px, py, pz + z_offset, rx, ry, rz)

    print("2. Bajando a recoger (Pick)...")
    robot.move_pose_euler(px, py, pz, rx, ry, rz)

    print("3. Cerrando garra...")
    robot.set_gripper(close_gripper=True)
    time.sleep(0.5) # Pausa dramatica para la simulacion

    print("4. Levantando objeto...")
    robot.move_pose_euler(px, py, pz + z_offset, rx, ry, rz)

    print("5. Moviendo a zona de destino (Approach Place)...")
    robot.move_pose_euler(dx, dy, dz + z_offset, rx, ry, rz)

    print("6. Bajando para dejar (Place)...")
    robot.move_pose_euler(dx, dy, dz, rx, ry, rz)

    print("7. Abriendo garra...")
    robot.set_gripper(close_gripper=False)
    time.sleep(0.5)

    print("8. Retirandose...")
    robot.move_pose_euler(dx, dy, dz + z_offset, rx, ry, rz)
    print(f"--- Fin secuencia de {nombre_obj} ---\n")


# --- EJECUCIÓN PRINCIPAL ---
if __name__ == "__main__":
    robot = RobotVisionController()

    if robot.connect():
        # 1. Posicion inicial segura
        robot.set_gripper(close_gripper=False) # Asegurar que esta abierta al iniciar
        robot.go_home()


        # 2. Definir coordenadas en MILIMETROS
        cubo_pick = [323.38, 58.88, 38.31]
        cubo_place = [-1.58, 274.70, 46.57]

        cilindro_pick = [278.83, -23.65, 45.69]
        cilindro_place = [14.77, -279.44, 45.69]

        # 3. Ejecutar las misiones usando nuestra funcion encapsulada
        ejecutar_pick_and_place(robot, "CUBO", cubo_pick, cubo_place, z_offset=80.0)

        # Volvemos a home entre tareas si es necesario, o vamos directo al siguiente
        robot.go_home()

        ejecutar_pick_and_place(robot, "CILINDRO", cilindro_pick, cilindro_place, z_offset=80.0)

        # 4. Finalizar
        # robot.go_home()
        robot.disconnect()
