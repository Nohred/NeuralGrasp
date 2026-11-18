# experiments — índice

## Qué es

Cada carpeta es un experimento autocontenido, con su propio README (Qué es / Requisitos / Cómo ejecutar / Qué esperar / Limitaciones / Siguiente paso). Los comandos se ejecutan **desde la raíz del repo**, salvo el 03, que corre desde el fork.

| id | Objetivo | RobotStudio | Cámara | GPU | Comando | Estado |
|---|---|:-:|:-:|:-:|---|---|
| [01_robot_ping](01_robot_ping/) | Conectividad Python ↔ RAPID (`PING`, `GET_POSE`) | ✔ (`test.mod`) | — | — | `python experiments/01_robot_ping/test.py` | probado por el usuario (según CLAUDE.md); sin verificar tras moverlo |
| [02_robot_pick_place](02_robot_pick_place/) | Pick & place de cubo y cilindro con coordenadas de jog | ✔ (`comm.mod`) | — | — | `python experiments/02_robot_pick_place/run.py` | probado por el usuario (según CLAUDE.md); secuencia verificada contra el servidor falso por Claude Code |
| [03_grasp_net_live](03_grasp_net_live/) | Predicción de agarres en vivo con GR-ConvNet, sin robot | — | ✔ (RealSense o webcam) | recomendada | `cd NeuralModel/robotic-grasping; python run_affordance_generator.py` | probado por el usuario en Linux (según CLAUDE.md); sin verificar en Windows |
| [04_handeye_synthetic](04_handeye_synthetic/) | Calibración mano-ojo eye-to-hand con datos sintéticos: ruido, número de poses y casos degenerados | opcional (stub) | — | — | `python experiments/04_handeye_synthetic/run.py` | **ejecutado por Claude Code** (200 reps) |

## Requisitos

- **01 y 02:** Python 3 (librería estándar) y RobotStudio con el módulo indicado.
- **03:** dependencias del fork (torch, …).
- **04:** entorno conda `neuralgrasp` (ver `requirements.txt` de la raíz).

## Cómo ejecutar

Ver el README de cada experimento.

## Qué esperar

Los resultados generados (`results/`) están ignorados por git; se regeneran con el comando de cada experimento.

## Limitaciones y problemas conocidos

Ver [docs/known_issues.md](../docs/known_issues.md).

## Siguiente paso

Cuando estén el robot físico y la cámara: un experimento 05 de calibración con datos reales (ChArUco + poses del robot), y después un 06 de "error de calibración vs. éxito de agarre" usando `calibration.perturb_transform`.
