# tests — pruebas automáticas sin hardware

## Qué es

Pruebas con pytest que **no** necesitan RobotStudio, cámara ni GPU.

| Archivo | Qué prueba |
|---|---|
| `test_transforms.py` | Ida y vuelta de `invert_T`, cuaternión RAPID (`[w,x,y,z]`) y Euler `OrientZYX`; orden del cuaternión; `pHome`; ángulo geodésico; **magnitud exacta** de `perturb_transform` (traslación y rotación, marcos `local`/`parent`) |
| `test_synthetic.py` | Marco de la cámara con convención OpenCV; **determinismo con semilla**; **filtro de visibilidad** (umbral de 60°, tablero de espaldas, distancia 300–1000 mm); cadena exacta sin ruido; estadística del ruido; modo de solo traslación |
| `test_handeye.py` | Recuperación sin ruido con **Tsai, Park, Daniilidis**, Horaud y Andreff (tolerancia 1e-6 mm/°); **test negativo**: sin invertir las poses el resultado es incorrecto; residuo de consistencia; validación de entradas |
| `test_rapid_client.py` | `RobotVisionController` contra `fake_rapid_server.py`, que imita las respuestas de `comm.mod`; `get_pose()` tras `HOME` y `MOVE_POSE`; secuencia de pick & place mensaje a mensaje; límite de 80 caracteres (también para la respuesta de `GET_POSE`) |
| `test_experiment04.py` | Prueba de humo de `experiments/04_handeye_synthetic/run.py` (modos synthetic y csv); modo `robotstudio` completo contra el servidor falso (`MOVE_POSE` → `GET_POSE` → calibración) y `--rs-dry-run`; `record_poses.py`; lector de CSV |
| `fake_rapid_server.py` | Servidor TCP falso (no es un test). Guarda la pose actual: `HOME`, `MOVE_POSE` (con `OrientZYX`) y `GRASP` la cambian, y `GET_POSE` la devuelve |

## Requisitos

Entorno conda `neuralgrasp` (ver `requirements.txt` en la raíz).

## Cómo ejecutar

Desde la raíz del repo:

```powershell
conda activate neuralgrasp
python -m pytest            # usa pytest.ini: testpaths = tests, pythonpath = .
python -m pytest -v tests/test_handeye.py
```

El test heredado del fork se corre aparte, desde su carpeta:

```powershell
cd NeuralModel/robotic-grasping
python -m pytest tests -rs   # 1 skipped: hardware.camera_factory no existe
```

## Qué esperar

**Ejecutado por Claude Code:** `96 passed` en ~5 s. El test del fork da `1 skipped`, con la razón.

## Limitaciones y problemas conocidos

- El servidor falso imita el **formato** de `comm.mod`, no la cinemática ni los errores de RAPID.
- `tests/test_camera_factory.py` del fork sigue roto, solo que ahora se omite (ver `docs/known_issues.md`, N3).

## Siguiente paso

Cuando haya detecciones reales de ChArUco, añadir un test con un conjunto pequeño de datos grabados (poses + detecciones) y comprobar que el residuo de consistencia está por debajo de un umbral.
