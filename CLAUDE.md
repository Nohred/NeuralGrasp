# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Estado actual del repo (reorganización del 2026-11-17)

> Las secciones de más abajo se escribieron **antes** de la reorganización. Se conservan, pero sus rutas y algunos datos están desactualizados: las notas **[ACTUALIZADO]** indican qué cambió. Esta sección manda.

- **Ahora sí es un repositorio git** (remoto `https://github.com/Nohred/NeuralGrasp.git`). Hay que trabajar en ramas y hacer un commit por cambio lógico. git está en `C:\Program Files\Git\cmd` (puede no estar en el PATH de la sesión).
- **Python:** entorno conda **`neuralgrasp`** (Python 3.11, numpy, scipy, opencv-python, matplotlib, pytest) para `calibration/`, `experiments/04` y `tests/`. Ejecutar con `conda run -n neuralgrasp python ...` o tras `conda activate neuralgrasp`. **No** llamar a `...\envs\neuralgrasp\python.exe` directamente: sin activar el entorno, numpy/MKL se cae (`0xc06d007f`). `conda run` no admite `python -c` multilínea; para eso, escribir un script. El canal `defaults` requiere aceptar los ToS de Anaconda: crear entornos con `--override-channels -c conda-forge`.
- **Dependencias del fork (torch, …) NO están instaladas** en `neuralgrasp`.

### Mapa del repo

```
README.md, CLAUDE.md, requirements.txt (calibration+tests), pytest.ini (testpaths=tests, pythonpath=.)
docs/coordinate_frames.md   convenciones (mm/grados, T_a_b, cuaternión RAPID [w,x,y,z], OrientZYX, OpenCV)
docs/known_issues.md        problemas detectados SIN corregir (R1-R12 robot, N1-N12 red, E1-E2 entorno)
docs/references/            1909.04810v4.pdf (GR-ConvNet), Actualización_Manual_Jetson.pdf
robot/rapid/                comm.mod, test.mod, points.mod   (antes en la raíz)
robot/client/rapid_client.py  RobotVisionController         (antes la clase de robot_comms.py)
RobotStudio/                estaciones .rsstn (no se movieron: estaban abiertas en RobotStudio)
NeuralModel/README.md       qué es original vs. añadido; dónde están pesos/datos; local_changes_vs_upstream.patch
NeuralModel/robotic-grasping/  fork (upstream skumra/robotic-grasping @ 183c6f6), sin reestructurar
  local_paths.py            rutas de checkpoints centralizadas (--network, GRCONVNET_NETWORK, GRCONVNET_ROOT)
calibration/                transforms, config, synthetic, handeye, metrics (mm, grados, 4x4)
experiments/01_robot_ping/test.py            (antes test.py)
experiments/02_robot_pick_place/run.py       (antes la parte de secuencias de robot_comms.py)
experiments/03_grasp_net_live/README.md      (apunta a run_affordance_generator.py / visualize_grasp_full.py)
experiments/04_handeye_synthetic/run.py      calibración eye-to-hand sintética (Monte Carlo)
tests/                      pytest sin hardware + fake_rapid_server.py (imita comm.mod)
```

Fuera del repo (`D:\Recuperacion\Escuela\9no Semestre\RobotVision_fuera_del_repo\`): manuales del IRB 1100 y el `.git` original del fork. Ignorados por git: dataset (`data/`), checkpoints (`epoch_*_iou_*`), eventos de TensorBoard y `results/`.

### Comandos

```powershell
conda activate neuralgrasp
python -m pytest                                        # 96 tests, sin hardware
python experiments/04_handeye_synthetic/run.py          # --poses synthetic|csv|robotstudio, --n-reps, --seed, --methods, --out
python experiments/04_handeye_synthetic/run.py --poses csv --csv experiments/04_handeye_synthetic/poses_example.csv
python experiments/04_handeye_synthetic/run.py --poses robotstudio   # mueve el robot con MOVE_POSE y lee GET_POSE
python experiments/04_handeye_synthetic/record_poses.py --out <csv>  # Enter = guardar GET_POSE (flujo con jog)
python experiments/02_robot_pick_place/run.py           # con comm.mod en RobotStudio
python experiments/01_robot_ping/test.py                # con test.mod en RobotStudio
cd NeuralModel/robotic-grasping; python run_affordance_generator.py --network <checkpoint>
```

### Calibración mano-ojo (eye-to-hand)

- `calibration.calibrate_eye_to_hand(T_base_gripper_list, T_cam_target_list, method)` → `T_base_cam`. Internamente pasa **las inversas** de las poses del robot a `cv2.calibrateHandEye` (que está planteada para cámara en la mano). No "arreglar" eso: hay un test negativo que demuestra que sin invertir el resultado es incorrecto.
- "gripper" = TCP de `tVision`. El tablero va a 50 mm sobre z de la herramienta (escena sintética).
- El robot del simulador es el **IRB 1100 de 475 mm** de alcance (confirmado por el usuario).
- **`GET_POSE` en `comm.mod`** (añadido el 2026-11-17, rama `feature/get-pose`, **sin verificar en RobotStudio**): `GET_POSE` → `POSE,x,y,z,q1,q2,q3,q4` (`CRobT` de `tVision` en `wobj0`; mm con 2 decimales y cuaternión RAPID con 6; ≤ 71 caracteres). Cliente: `RobotVisionController.get_pose()`. Es el único cambio de protocolo; los demás comandos no cambiaron.
- Cubo del pick & place: la pose válida es la del script (`z = 38.31`); `points.mod` se ignora para el cubo (decisión del usuario).
- Daniilidis (OpenCV) tiene fallos catastróficos raros (~1% con ruido, error de ~180°); el residuo de consistencia los detecta sin verdad de terreno.

### Correcciones a lo de abajo

- La pinza usa la señal **`GripperOpen`**; `SetDO do_Gripper` está **comentado** en `comm.mod` (al revés de lo que decía la línea de `GRIPPER`).
- **Solo `MOVE_POSE`** clona `CRobT`; `GRASP` no (su `robconf` y `extax` quedan en 0).
- Los scripts usan el checkpoint `epoch_48_iou_0.92` (el de mejor IoU es `epoch_20_iou_0.93`).
- Las rutas de Linux ya no están escritas a mano en los scripts: salen de `local_paths.py` (mismo valor por defecto).
- `tests/test_camera_factory.py` ya no falla: se omite (skip) con la razón.

## Contexto del proyecto

Proyecto escolar (Robot Vision): agarre robótico guiado por visión. Hay dos mitades que **todavía no están conectadas entre sí**:

1. **Control del robot (raíz del repo)**: un servidor TCP en RAPID que corre en un controlador virtual de RobotStudio y un cliente en Python que le manda comandos.
2. **Red neuronal (`NeuralModel/robotic-grasping/`)**: fork de GR-ConvNet (Kumra et al., paper `1909.04810v4.pdf`) que predice agarres antipodales a partir de imágenes RGB/RGB-D.

No es un repositorio git. No hay build, linter ni CI. **[ACTUALIZADO]** Ahora sí es un repo git; sigue sin haber build, linter ni CI (hay pytest).

## Entorno del robot

- RobotStudio con controlador virtual **OmniCore C30**, **RobotWare 7.21.5**, robot **IRB 1100**. Estaciones en `RobotStudio/` (`NeuralGrasp.rsstn`, `NeuralTest.rsstn`).
- Los `.mod` se cargan a mano en la tarea T_ROB1 desde RobotStudio (RAPID → Load Module); no hay forma de cargarlos desde la terminal. Después de editar un `.mod` el usuario tiene que recargarlo y reiniciar el puntero a `main`.
- Manuales del IRB 1100 en la raíz (`3HAC064992 PM ...` manual de producto, `3HAC064993 PS ...` especificaciones), útiles para alcance y límites de ejes. **[ACTUALIZADO]** Movidos fuera del repo, a `D:\Recuperacion\Escuela\9no Semestre\RobotVision_fuera_del_repo\`.
- `Actualización_Manual_Jetson.pdf` (**[ACTUALIZADO]** ahora en `docs/references/`): instrucciones para flashear JetPack 6.2 + ZED SDK en una Jetson Orin Nano (plataforma de destino prevista para la visión; aún no hay código para ella).

## Protocolo de sockets RAPID ↔ Python

- `comm.mod` (módulo `ServidorVision`) es el servidor **activo**; `test.mod` (`ServidorPython`, comandos `PING`/`GET_POSE`) y `test.py` son la prueba inicial de conectividad. Ambos módulos definen `main` y usan el mismo puerto, así que **solo uno puede estar cargado a la vez**.
- Servidor en `127.0.0.1:55555`. Al aceptar la conexión manda un saludo, que el cliente debe leer con `recv` antes de enviar nada.
- Request/response síncrono, texto ASCII sin delimitador: un `sendall` → un `recv`. RAPID recibe en un `string` (máx. 80 caracteres), así que los mensajes deben ser cortos y no deben enviarse varios comandos sin esperar la respuesta.
- Comandos (`CMD,v1,v2,...`, mm y grados, respecto a `wobj0` con la herramienta `tVision` cuyo TCP está en z=100 mm):
  - `HOME` → `MoveJ pHome`.
  - **[ACTUALIZADO]** `GET_POSE` → `POSE,x,y,z,q1,q2,q3,q4` del TCP `tVision` en `wobj0` (comando nuevo; ver la sección de arriba).
  - `MOVE_POSE,x,y,z,rx,ry,rz` → `MoveL` con `OrientZYX(rz, ry, rx)`.
  - `GRASP,x,y,z,?,ang` → `MoveL` con orientación fija boca abajo y giro `ang` en Z (el 4.º valor se parsea pero no se usa). Aún no hay método en el cliente Python.
  - `GRIPPER,1|0` → `SetDO do_Gripper`. **La señal `do_Gripper` debe existir en la configuración de I/O del controlador**, si no, el módulo no carga (los comentarios "Descomentar..." están desactualizados: las líneas ya están activas). **[ACTUALIZADO]** Incorrecto: en `comm.mod` las líneas de `do_Gripper` están **comentadas** y se usa `GripperOpen` (`SetDO GripperOpen, 0` / `Set GripperOpen`). La señal que debe existir es `GripperOpen` (`docs/known_issues.md`, R1).
- Respuestas: `OK`, `ERR_FORMATO`, `ERR_CMD_NO_RECONOCIDO`, `ERR_SINTAXIS`. Al cerrar el cliente, el `ERROR` handler con `ERR_SOCK_CLOSED` vuelve a `SocketAccept`.
- `MOVE_POSE`/`GRASP` clonan `CRobT` para heredar `robconf`; un cambio grande de orientación puede provocar errores de configuración en `MoveL`. **[ACTUALIZADO]** Solo `MOVE_POSE` (y el inexistente `MOVE`) clonan `CRobT`; `GRASP` no (R2).
- `points.mod` contiene robtargets obtenidos con jog (home, pick de cubo/cilindro, place). `robot_comms.py` replica esas coordenadas como listas en Python. **[ACTUALIZADO]** `points.mod` no tiene `MODULE…ENDMODULE`, y su cubo (z = 62.77) no coincide con el script (z = 38.31) (R8, R9).

Ejecutar el cliente (con el programa RAPID en marcha en RobotStudio):

```
python robot_comms.py   # pick & place de cubo y cilindro
python test.py          # solo con test.mod cargado
```

**[ACTUALIZADO]** Rutas nuevas: `python experiments/02_robot_pick_place/run.py` y `python experiments/01_robot_ping/test.py`.

## NeuralModel/robotic-grasping

Ver su `README.md` para entrenamiento/evaluación. Ejecutar todo **desde dentro de `NeuralModel/robotic-grasping/`** (los imports son relativos a esa carpeta: `hardware`, `inference`, `utils`).

```
pip install -r requirements.txt
python train_network.py --dataset cornell --dataset-path data/archive --network grconvnet3 --use-depth 0 --use-rgb 1 --input-size 300
python evaluate.py --network <checkpoint> --dataset cornell --dataset-path data/archive --iou-eval
python run_affordance_generator.py   # inferencia en vivo con cámara + overlay OpenCV
pytest tests                          # ver nota abajo
```

Arquitectura:
- `inference/models/` define las redes (`grconvnet3` es la usada). Los checkpoints se guardan con `torch.save(model)` completo (archivos sin extensión `epoch_XX_iou_Y`), se cargan con `torch.load` y se infiere con `net.predict(x)` → `pos/cos/sin/width`.
- Flujo: cámara → `utils/data/camera_data.CameraData` (recorte central a `input_size`, normalización) → red → `inference/post_process.post_process_output` → `utils/dataset_processing/grasp.detect_grasps` (centro en píxeles, ángulo, ancho).
- `hardware/camera.py` (RealSense) y `hardware/camera_web.py` (webcam, añadido localmente) exponen la misma API (`connect`, `get_image_bundle` → `rgb`/`aligned_depth`, `intrinsics`). La webcam genera una "profundidad" sintética a partir de la luminancia: no es métrica.
- `inference/grasp_generator.py` convierte el agarre de píxel a coordenadas del robot usando `saved_data/camera_pose.txt` (4x4 cámara→robot) y `camera_depth_scale.txt` (de `run_calibration.py`), y se comunica por archivos `.npy` en `~/grasp-comms` (heredado del repo original para Baxter). Aquí está el punto natural de unión con `robot_comms.py`.
- Modelo entrenado localmente: `logs/260831_1231_training_cornell_rgb/` (Cornell, solo RGB, input 300, mejor `epoch_20_iou_0.93`). Modelos preentrenados en `trained-models/`. Dataset Cornell en `data/archive/`.

Advertencias conocidas:
- Las rutas de modelos están hardcodeadas con rutas Linux (`/media/ulises-calderon/SteamDisk/...`) en `run_affordance_generator.py` y `run_grasp_generator.py`; en Windows el repo vive en `D:\Recuperacion\...`. **[ACTUALIZADO]** Centralizadas en `local_paths.py` (también las de `visualize_grasp_full.py`). En Windows: `$env:GRCONVNET_ROOT = "D:\...\NeuralModel\robotic-grasping"` o `--network <ruta>`.
- `run_affordance_generator.py` usa `INPUT_SIZE = 224` mientras el modelo RGB se entrenó con 300; el `input_size` y `use_depth/use_rgb` deben coincidir con los del checkpoint (ver `commandline_args.json` de cada run).
- `tests/test_camera_factory.py` importa `hardware.camera_factory`, que no existe: el test falla hasta crear ese módulo. **[ACTUALIZADO]** Ahora se omite (skip a nivel de módulo, con la razón). El módulo sigue sin existir.
- Lista completa de problemas, sin corregir: `docs/known_issues.md`.
