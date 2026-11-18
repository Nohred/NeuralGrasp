# NeuralGrasp — agarre robótico guiado por visión (IRB 1100 + GR-ConvNet)

## Qué es

Proyecto de tesis de licenciatura: un **ABB IRB 1100** (controlador OmniCore C30, RobotWare 7.21.5), por ahora solo en **RobotStudio**, comandado desde Python por sockets, y una red **GR-ConvNet** que predice puntos de agarre a partir de imágenes. La meta a largo plazo es estudiar **cómo el error de calibración mano-ojo afecta el éxito del agarre**, con cámara fija (eye-to-hand) y un tablero ChArUco montado en el gripper.

Las dos mitades (robot y red) **todavía no están conectadas**. El robot físico aún no está disponible.

## Mapa del repo

```
README.md                    este archivo
CLAUDE.md                    guía para Claude Code (contexto, comandos, advertencias)
requirements.txt             dependencias de calibration/ y tests/
pytest.ini                   configuración de pytest (solo tests/)
docs/
  coordinate_frames.md       unidades, marcos, T_a_b, cuaterniones RAPID vs scipy, OrientZYX, OpenCV
  known_issues.md            problemas detectados y SIN corregir (archivo, línea, efecto, arreglo)
  references/                paper GR-ConvNet (1909.04810v4.pdf), manual Jetson Orin Nano + ZED
robot/
  rapid/                     comm.mod (servidor activo), test.mod (ping), points.mod (puntos de jog)
  client/                    rapid_client.py: RobotVisionController (cliente de sockets)
RobotStudio/                 estaciones NeuralGrasp.rsstn y NeuralTest.rsstn
NeuralModel/
  README.md                  qué es del repo original y qué se añadió; dónde están pesos y datos
  robotic-grasping/          fork de GR-ConvNet (sin reestructurar)
calibration/                 librería de calibración mano-ojo eye-to-hand (sin hardware)
experiments/
  01_robot_ping/             conectividad (test.mod + test.py)
  02_robot_pick_place/       pick & place de cubo y cilindro (comm.mod)
  03_grasp_net_live/         predicción en vivo de la red, sin robot
  04_handeye_synthetic/      calibración mano-ojo con datos sintéticos (Monte Carlo)
tests/                       pytest sin hardware (incluye un servidor falso de comm.mod)
```

Fuera del repo, en `D:\Recuperacion\Escuela\9no Semestre\RobotVision_fuera_del_repo\`, quedaron los manuales del IRB 1100 (`3HAC064992`, `3HAC064993`) y el `.git` original del fork. Los pesos y el dataset siguen en su sitio, ignorados por git (ver `NeuralModel/README.md`).

## Requisitos

| Parte | Requisitos |
|---|---|
| Robot (`robot/`, experimentos 01-02) | RobotStudio + controlador virtual; Python 3 (librería estándar) |
| Red (`NeuralModel/`, experimento 03) | `NeuralModel/robotic-grasping/requirements.txt` (torch, …), cámara, GPU recomendada |
| Calibración (`calibration/`, experimento 04, `tests/`) | conda env `neuralgrasp`: numpy, scipy, opencv-python, matplotlib, pytest |

## Cómo ejecutar

```powershell
# Entorno para calibration/ y tests/ (una sola vez)
conda create -n neuralgrasp --override-channels -c conda-forge python=3.11 numpy scipy matplotlib pytest
conda activate neuralgrasp
pip install opencv-python

# Tests sin hardware
python -m pytest

# Experimento 04 (sin hardware)
python experiments/04_handeye_synthetic/run.py

# Robot (con comm.mod cargado y en marcha en RobotStudio)
python experiments/02_robot_pick_place/run.py

# Red en vivo (desde el fork)
cd NeuralModel/robotic-grasping; python run_affordance_generator.py
```

## Qué esperar / estado

| Parte | Estado |
|---|---|
| `calibration/` + `tests/` | **ejecutado por Claude Code**: 90 tests pasan |
| Experimento 04 | **ejecutado por Claude Code**: la prueba sin ruido da OK con los 5 métodos (~1e-11); Monte Carlo de 200 reps |
| Cliente de sockets | **ejecutado por Claude Code** contra el servidor falso; sin verificar contra RobotStudio tras la reorganización |
| `comm.mod`, pick & place, ping | probado por el usuario (según CLAUDE.md) |
| Red: entrenamiento e inferencia en vivo | probado por el usuario en Linux (según CLAUDE.md); sin verificar en Windows |
| Convención `OrientZYX` | sale del manual de RAPID y está probada en Python; sin verificar en RobotStudio |

## Limitaciones y problemas conocidos

Ver [docs/known_issues.md](docs/known_issues.md). Lo más relevante: la pinza usa `GripperOpen` y no `do_Gripper`; `GRASP` no clona `CRobT` y no tiene cliente; `INPUT_SIZE` 224 con un checkpoint de 300; `camera_pose.txt` sin calibrar; unidades mezcladas (m vs. mm) en el código heredado de la red.

## Siguiente paso

1. Añadir a `comm.mod` un `GET_POSE` con `x,y,z,q1..q4` de `tVision`, para grabar poses de calibración.
2. Detectar ChArUco con la cámara real y calibrar con `calibration.calibrate_eye_to_hand`, juzgando el resultado con el residuo de consistencia.
3. Integrar red → `T_base_cam` → comando `GRASP`, y montar el experimento "error de calibración vs. éxito de agarre" con `perturb_transform`.
