# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Contexto del proyecto

Proyecto escolar (Robot Vision): agarre robótico guiado por visión. Hay dos mitades que **todavía no están conectadas entre sí**:

1. **Control del robot (raíz del repo)**: un servidor TCP en RAPID que corre en un controlador virtual de RobotStudio y un cliente en Python que le manda comandos.
2. **Red neuronal (`NeuralModel/robotic-grasping/`)**: fork de GR-ConvNet (Kumra et al., paper `1909.04810v4.pdf`) que predice agarres antipodales a partir de imágenes RGB/RGB-D.

No es un repositorio git. No hay build, linter ni CI.

## Entorno del robot

- RobotStudio con controlador virtual **OmniCore C30**, **RobotWare 7.21.5**, robot **IRB 1100**. Estaciones en `RobotStudio/` (`NeuralGrasp.rsstn`, `NeuralTest.rsstn`).
- Los `.mod` se cargan a mano en la tarea T_ROB1 desde RobotStudio (RAPID → Load Module); no hay forma de cargarlos desde la terminal. Después de editar un `.mod` el usuario tiene que recargarlo y reiniciar el puntero a `main`.
- Manuales del IRB 1100 en la raíz (`3HAC064992 PM ...` manual de producto, `3HAC064993 PS ...` especificaciones), útiles para alcance y límites de ejes.
- `Actualización_Manual_Jetson.pdf`: instrucciones para flashear JetPack 6.2 + ZED SDK en una Jetson Orin Nano (plataforma de destino prevista para la visión; aún no hay código para ella).

## Protocolo de sockets RAPID ↔ Python

- `comm.mod` (módulo `ServidorVision`) es el servidor **activo**; `test.mod` (`ServidorPython`, comandos `PING`/`GET_POSE`) y `test.py` son la prueba inicial de conectividad. Ambos módulos definen `main` y usan el mismo puerto, así que **solo uno puede estar cargado a la vez**.
- Servidor en `127.0.0.1:55555`. Al aceptar la conexión manda un saludo, que el cliente debe leer con `recv` antes de enviar nada.
- Request/response síncrono, texto ASCII sin delimitador: un `sendall` → un `recv`. RAPID recibe en un `string` (máx. 80 caracteres), así que los mensajes deben ser cortos y no deben enviarse varios comandos sin esperar la respuesta.
- Comandos (`CMD,v1,v2,...`, mm y grados, respecto a `wobj0` con la herramienta `tVision` cuyo TCP está en z=100 mm):
  - `HOME` → `MoveJ pHome`.
  - `MOVE_POSE,x,y,z,rx,ry,rz` → `MoveL` con `OrientZYX(rz, ry, rx)`.
  - `GRASP,x,y,z,?,ang` → `MoveL` con orientación fija boca abajo y giro `ang` en Z (el 4.º valor se parsea pero no se usa). Aún no hay método en el cliente Python.
  - `GRIPPER,1|0` → `SetDO do_Gripper`. **La señal `do_Gripper` debe existir en la configuración de I/O del controlador**, si no, el módulo no carga (los comentarios "Descomentar..." están desactualizados: las líneas ya están activas).
- Respuestas: `OK`, `ERR_FORMATO`, `ERR_CMD_NO_RECONOCIDO`, `ERR_SINTAXIS`. Al cerrar el cliente, el `ERROR` handler con `ERR_SOCK_CLOSED` vuelve a `SocketAccept`.
- `MOVE_POSE`/`GRASP` clonan `CRobT` para heredar `robconf`; un cambio grande de orientación puede provocar errores de configuración en `MoveL`.
- `points.mod` contiene robtargets obtenidos con jog (home, pick de cubo/cilindro, place). `robot_comms.py` replica esas coordenadas como listas en Python.

Ejecutar el cliente (con el programa RAPID en marcha en RobotStudio):

```
python robot_comms.py   # pick & place de cubo y cilindro
python test.py          # solo con test.mod cargado
```

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
- Las rutas de modelos están hardcodeadas con rutas Linux (`/media/ulises-calderon/SteamDisk/...`) en `run_affordance_generator.py` y `run_grasp_generator.py`; en Windows el repo vive en `D:\Recuperacion\...`.
- `run_affordance_generator.py` usa `INPUT_SIZE = 224` mientras el modelo RGB se entrenó con 300; el `input_size` y `use_depth/use_rgb` deben coincidir con los del checkpoint (ver `commandline_args.json` de cada run).
- `tests/test_camera_factory.py` importa `hardware.camera_factory`, que no existe: el test falla hasta crear ese módulo.
