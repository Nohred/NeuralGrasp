# calibration — calibración mano-ojo eye-to-hand (sin hardware)

## Qué es

Librería para calibrar una cámara **fija** respecto a la base del robot (`T_base_cam`), con un tablero ChArUco montado en el gripper. Todo está en **mm, grados y matrices 4x4** (`T_a_b` = pose de `b` en `a`; ver [docs/coordinate_frames.md](../docs/coordinate_frames.md)). La interfaz recibe **listas de arrays 4x4**, así que después se pueden enchufar poses reales del robot y detecciones reales de ChArUco sin reescribir nada.

| Módulo | Contenido |
|---|---|
| `transforms.py` | `make_T`, `invert_T`, cuaternión RAPID ↔ R (`[w,x,y,z]`), Euler de `MOVE_POSE` ↔ R (`OrientZYX`), `rotation_angle_deg` (geodésico), `perturb_transform(T, trans_mm, rot_deg, axis, frame)` |
| `config.py` | `SceneConfig` (escena por defecto) y `workspace_box(reach_mm)` para el IRB 1100 de 475 o 580 mm |
| `synthetic.py` | `SyntheticScene`: verdad de terreno, muestreo de poses con semilla, filtro de visibilidad y detecciones con ruido |
| `handeye.py` | `calibrate_eye_to_hand(T_base_gripper_list, T_cam_target_list, method)` → `T_base_cam`; `METHODS` |
| `metrics.py` | `pose_error` (vs. verdad), `consistency_residual` (**sin** verdad de terreno) y `summarize` |

### La trampa de `cv2.calibrateHandEye` con cámara fija

`cv2.calibrateHandEye` está planteada para cámara **en la mano**. Con cámara fija hay que pasarle las **inversas** de las poses del robot (`T_gripper_base = inv(T_base_gripper)`) como `gripper2base`; el resultado (`cam2gripper`) se lee como `T_base_cam`. La derivación está en el docstring de `handeye.py`. Lo comprueban dos tests: sin ruido, el error es ~1e-11; sin invertir, el error es de ~1000 mm.

### Escena sintética por defecto (suposiciones)

- **Cámara:** en `(1100, 0, 500)` mm, mirando hacia `(290, 0, 275)` mm, a ~0.84 m de la zona de trabajo.
- **Tablero:** a 50 mm sobre el eje z del TCP, con la cara impresa hacia +z de la herramienta.
- **Caja de trabajo:** para 475 mm, aprox. x∈[200, 380], y∈[-150, 150], z∈[140, 400] mm, más un filtro aproximado de alcance respecto al hombro. **No** se resuelve cinemática inversa.
- **Orientaciones:** la herramienta apunta a la cámara con una inclinación aleatoria de hasta 40° y un giro de hasta ±90°.
- **Visibilidad:** se descarta la pose si la normal del tablero está a más de 60° de la línea de visión, si el tablero queda detrás de la cámara o si está fuera de 300–1000 mm.
- **Ruido de la detección:** traslación ~ N(0, σt²) por eje, en el marco de la cámara; rotación con rotvec ~ N(0, σr²) por eje, alrededor del origen del tablero.

## Requisitos

`numpy`, `scipy`, `opencv-python`, `matplotlib` (para el experimento) y `pytest`. Ver `requirements.txt` en la raíz.

```powershell
conda create -n neuralgrasp --override-channels -c conda-forge python=3.11 numpy scipy matplotlib pytest
conda activate neuralgrasp
pip install opencv-python
```

## Cómo ejecutar

Desde la raíz del repo, con `conda activate neuralgrasp`:

```python
import numpy as np
from calibration import SyntheticScene, calibrate_eye_to_hand, pose_error, consistency_residual

scene = SyntheticScene.from_config()
rng = np.random.default_rng(0)
poses = scene.sample_gripper_poses(15, rng)                 # T_base_gripper (4x4, mm)
dets = scene.detect(poses, rng, noise_trans_mm=1.0, noise_rot_deg=0.25)   # T_cam_target
T_base_cam = calibrate_eye_to_hand(poses, dets, "park")
print(pose_error(T_base_cam, scene.T_base_cam))             # (mm, grados)
print(consistency_residual(poses, dets, T_base_cam)["trans_rms_mm"])
```

Con datos reales sería lo mismo, cambiando `poses` por las poses del robot (`pose_rapid_to_T(x,y,z,q1,q2,q3,q4)`) y `dets` por las poses de ChArUco en mm.

```powershell
python -m pytest tests -q                                   # tests de la librería
python experiments/04_handeye_synthetic/run.py              # estudio Monte Carlo
```

## Qué esperar

**Ejecutado por Claude Code:**
- **Sin ruido:** los 5 métodos (Tsai, Park, Horaud, Andreff y Daniilidis) recuperan `T_base_cam` con error < 1e-10 mm / grados.
- **Sin invertir las poses:** el error es de ~1000 mm y ~130°.
- **Con ruido:** ver los resultados en `experiments/04_handeye_synthetic/README.md`.

## Limitaciones y problemas conocidos

- La escena es una **suposición**: posición de la cámara, montaje del tablero y caja de trabajo no son medidas reales. La altura del hombro (327 mm) es aproximada.
- No hay cinemática inversa ni detección de colisiones; una pose "válida" aquí podría no ser alcanzable con el TCP orientado así.
- El ruido es gaussiano e independiente por pose. El error real de ChArUco depende de la distancia y el ángulo, y el robot tiene su propio error de repetibilidad, que aquí no se modela.
- `R_to_euler_rapid` tiene bloqueo de cardán cerca de `ry = ±90°`.
- La convención `OrientZYX` sale del manual de RAPID y está probada en Python, pero **sin verificar en RobotStudio**.

## Siguiente paso

1. Detectar ChArUco con la cámara real y producir `T_cam_target` en mm (fuera de alcance aquí).
2. Grabar las poses reales del TCP (`tVision`) en CSV o con un `GET_POSE` completo en `comm.mod`.
3. Usar `consistency_residual` para juzgar la calibración real (no hay verdad de terreno).
4. Usar `perturb_transform` sobre `T_base_cam` para el experimento "error de calibración vs. éxito de agarre".
