"""Escena sintética eye-to-hand con verdad de terreno.

Marcos (ver docs/coordinate_frames.md):
  - base     : wobj0 del robot.
  - gripper  : TCP de la herramienta (tVision en comm.mod).
  - cam      : marco óptico de la cámara, convención OpenCV (x der., y abajo, z adelante).
  - target   : tablero ChArUco, convención OpenCV (origen en una esquina, z normal al
               tablero apuntando HACIA ATRÁS de la cara impresa).

Verdad de terreno: ``T_base_cam`` (fija) y ``T_gripper_target`` (fija).
Detección simulada: ``T_cam_target = inv(T_base_cam) @ T_base_gripper @ T_gripper_target``
más ruido gaussiano.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import SHOULDER_HEIGHT_MM, SceneConfig
from .transforms import axis_angle_to_R, invert_T, make_T, random_rotation_small


def look_at(cam_position, target, world_up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """``T_base_cam`` de una cámara en ``cam_position`` que mira hacia ``target``.

    Convención OpenCV: z_cam hacia el objetivo, x_cam a la derecha de la imagen,
    y_cam hacia abajo (el "arriba" de la imagen es ``world_up``).
    """
    p = np.asarray(cam_position, dtype=float)
    z = np.asarray(target, dtype=float) - p
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(world_up, dtype=float))
    if np.linalg.norm(x) < 1e-9:
        raise ValueError("La dirección de vista es paralela a world_up")
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return make_T(np.column_stack([x, y, z]), p)


def board_on_tool(offset_mm: float) -> np.ndarray:
    """``T_gripper_target``: tablero a ``offset_mm`` sobre el eje z de la herramienta.

    La cara impresa mira en +z de la herramienta, así que el eje z del tablero
    (convención OpenCV) es -z de la herramienta: rotación de 180° en x.
    """
    return make_T(axis_angle_to_R("x", 180.0), [0.0, 0.0, offset_mm])


@dataclass
class SyntheticScene:
    T_base_cam: np.ndarray
    T_gripper_target: np.ndarray
    config: SceneConfig

    @classmethod
    def from_config(cls, config: SceneConfig | None = None) -> "SyntheticScene":
        cfg = config or SceneConfig()
        return cls(
            T_base_cam=look_at(cfg.cam_position_mm, cfg.cam_look_at_mm),
            T_gripper_target=board_on_tool(cfg.board_offset_mm),
            config=cfg,
        )

    # ------------------------------------------------------------ geometría
    def T_cam_target_true(self, T_base_gripper) -> np.ndarray:
        """Pose exacta del tablero vista por la cámara."""
        return invert_T(self.T_base_cam) @ np.asarray(T_base_gripper) @ self.T_gripper_target

    def view_geometry(self, T_base_gripper):
        """Devuelve ``(distancia_mm, angulo_vista_deg, z_cam_mm)`` del tablero."""
        T = self.T_cam_target_true(T_base_gripper)
        p = T[:3, 3]
        dist = float(np.linalg.norm(p))
        # Ángulo entre la normal del tablero (su eje z, hacia atrás de la cara impresa)
        # y la línea de visión cámara->tablero. 0° = tablero de frente a la cámara.
        cosang = float(np.clip(T[:3, 2] @ (p / dist), -1.0, 1.0))
        return dist, float(np.degrees(np.arccos(cosang))), float(p[2])

    def is_visible(self, T_base_gripper) -> bool:
        """Filtro: tablero delante de la cámara, de frente (<= max_view_angle_deg)
        y a una distancia dentro de [min_distance_mm, max_distance_mm]."""
        c = self.config
        dist, ang, zc = self.view_geometry(T_base_gripper)
        return zc > 0 and ang <= c.max_view_angle_deg and c.min_distance_mm <= dist <= c.max_distance_mm

    def is_reachable(self, position) -> bool:
        """Filtro aproximado de alcance (distancia al hombro); NO es cinemática inversa."""
        shoulder = np.array([0.0, 0.0, SHOULDER_HEIGHT_MM])
        return np.linalg.norm(np.asarray(position) - shoulder) <= self.config.reach_margin * self.config.reach_mm

    # ------------------------------------------------------------- muestreo
    def _tool_orientation(self, position, rng, tilt_max_deg, roll_max_deg) -> np.ndarray:
        """Herramienta apuntando (eje z) hacia la cámara, con inclinación y giro aleatorios."""
        d = self.T_base_cam[:3, 3] - position
        d /= np.linalg.norm(d)
        # Eje de inclinación aleatorio perpendicular a d.
        v = rng.normal(size=3)
        v -= (v @ d) * d
        v /= np.linalg.norm(v)
        tilt = rng.uniform(0.0, tilt_max_deg) if tilt_max_deg > 0 else 0.0
        z = axis_angle_to_R(v, tilt) @ d
        # x de referencia horizontal, luego giro alrededor de z.
        x = np.cross([0.0, 0.0, 1.0], z)
        if np.linalg.norm(x) < 1e-9:
            x = np.array([1.0, 0.0, 0.0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        R0 = np.column_stack([x, y, z])
        roll = rng.uniform(-roll_max_deg, roll_max_deg) if roll_max_deg > 0 else 0.0
        return R0 @ axis_angle_to_R("z", roll)

    def sample_gripper_poses(
        self,
        n: int,
        rng: np.random.Generator,
        mode: str = "varied",
        low_rotation_deg: float = 3.0,
        max_tries: int = 100000,
    ) -> list:
        """Muestrea ``n`` poses ``T_base_gripper`` válidas (alcanzables y con tablero visible).

        Modos:
          - ``'varied'``: inclinación hasta ``tilt_max_deg`` y giro hasta ``roll_max_deg``.
          - ``'low_rotation'``: inclinación y giro de como máximo ``low_rotation_deg``.
          - ``'translation_only'``: MISMA orientación en todas las poses (caso degenerado:
            la rotación de la calibración mano-ojo no es observable).
        """
        c = self.config
        ws = c.workspace
        fixed_R = None
        if mode == "translation_only":
            center = ws.mean(axis=1)
            fixed_R = self._tool_orientation(center, rng, 0.0, 0.0)
        elif mode not in ("varied", "low_rotation"):
            raise ValueError(f"Modo desconocido: {mode!r}")

        poses = []
        for _ in range(max_tries):
            p = rng.uniform(ws[:, 0], ws[:, 1])
            if not self.is_reachable(p):
                continue
            if fixed_R is not None:
                R = fixed_R
            elif mode == "low_rotation":
                R = self._tool_orientation(p, rng, low_rotation_deg, low_rotation_deg)
            else:
                R = self._tool_orientation(p, rng, c.tilt_max_deg, c.roll_max_deg)
            T = make_T(R, p)
            if self.is_visible(T):
                poses.append(T)
                if len(poses) == n:
                    return poses
        raise RuntimeError(f"Solo se encontraron {len(poses)}/{n} poses válidas en {max_tries} intentos")

    # ----------------------------------------------------------- detecciones
    def detect(self, T_base_gripper_list, rng: np.random.Generator,
               noise_trans_mm: float = 0.0, noise_rot_deg: float = 0.0) -> list:
        """Detecciones falsas ``T_cam_target`` con ruido gaussiano.

        Ruido: traslación ~ N(0, noise_trans_mm^2) por eje, en el marco de la cámara;
        rotación con rotvec ~ N(0, noise_rot_deg^2) por eje, aplicada alrededor del
        origen del tablero (``R_ruidosa = R_ruido @ R``).
        """
        out = []
        for T_bg in T_base_gripper_list:
            T = self.T_cam_target_true(T_bg).copy()
            if noise_rot_deg > 0:
                T[:3, :3] = random_rotation_small(rng, noise_rot_deg) @ T[:3, :3]
            if noise_trans_mm > 0:
                T[:3, 3] = T[:3, 3] + rng.normal(0.0, noise_trans_mm, size=3)
            out.append(T)
        return out
