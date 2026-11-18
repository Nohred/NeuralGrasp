"""Valores por defecto de la escena sintética (mm y grados).

Marco base = ``wobj0`` del IRB 1100 (x hacia el frente del robot, z hacia arriba).
Todos los valores son SUPOSICIONES razonables para simular; no son medidas reales.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Alcance nominal según la variante del IRB 1100 (mm): 0.475 m o 0.58 m.
IRB1100_REACH_MM = {475: 475.0, 580: 580.0}

# Altura aproximada del eje 2 (hombro) sobre la base, en mm. Aproximación para el
# filtro de alcance; verificar en el manual de producto antes de usarla en serio.
SHOULDER_HEIGHT_MM = 327.0


def workspace_box(reach_mm: float) -> np.ndarray:
    """Caja de trabajo CONSERVADORA en el marco base, proporcional al alcance.

    Devuelve ``[[xmin, xmax], [ymin, ymax], [zmin, zmax]]`` en mm. Para 475 mm queda
    aprox. x∈[200, 380], y∈[-150, 150], z∈[140, 400]: zona frontal, lejos de los
    límites de alcance y del cuerpo del robot. No se resuelve cinemática inversa.
    """
    r = float(reach_mm)
    return np.array([[0.42 * r, 0.80 * r], [-0.32 * r, 0.32 * r], [0.30 * r, 0.85 * r]])


@dataclass
class SceneConfig:
    """Parámetros de la escena eye-to-hand (cámara fija, tablero en el gripper)."""

    # Variante del robot (alcance en mm) -> define la caja de trabajo.
    reach_mm: float = 475.0
    # Cámara fija ~1 m frente al robot, mirando hacia la zona de trabajo.
    cam_position_mm: tuple = (1100.0, 0.0, 500.0)
    cam_look_at_mm: tuple = (290.0, 0.0, 275.0)
    # Tablero (ChArUco) a 50 mm sobre el eje z de la herramienta (gripper = TCP tVision).
    board_offset_mm: float = 50.0
    # Filtro de visibilidad.
    max_view_angle_deg: float = 60.0      # normal del tablero vs. línea de visión
    min_distance_mm: float = 300.0
    max_distance_mm: float = 1000.0
    # Muestreo de orientaciones del gripper: la herramienta apunta hacia la cámara
    # con una inclinación aleatoria dentro de un cono y un giro (roll) aleatorio.
    tilt_max_deg: float = 40.0
    roll_max_deg: float = 90.0
    # Filtro de alcance: distancia máxima al hombro como fracción del alcance.
    reach_margin: float = 0.9
    workspace: np.ndarray = field(default=None)

    def __post_init__(self):
        if self.workspace is None:
            self.workspace = workspace_box(self.reach_mm)
        self.workspace = np.asarray(self.workspace, dtype=float)
