# Problemas conocidos (documentados, sin corregir)

Ninguno de estos problemas se corrigió en la reorganización, para no cambiar el comportamiento. La única excepción fueron las rutas fijas de Linux, que se **centralizaron** (ver al final). Las líneas se refieren a las rutas nuevas.

## Robot (RAPID / cliente)

| # | Archivo:línea | Problema | Efecto | Arreglo sugerido |
|---|---|---|---|---|
| R1 | `robot/rapid/comm.mod:23,83,89` y `:82,88` | La pinza usa la señal **`GripperOpen`**. `SetDO do_Gripper` está **comentado** (el CLAUDE.md original decía lo contrario). Los comentarios "Descomentar cuando configures la señal" siguen ahí. | Si `GripperOpen` no existe en la configuración de I/O del controlador, el módulo **no carga**. `do_Gripper` no se usa. | Decidir qué señal es la buena, dejar solo esa y borrar los comentarios desactualizados. |
| R2 | `robot/rapid/comm.mod:52-54, 66-70` | `GRASP` **no clona `CRobT`**: solo lo hacen `MOVE_POSE` y `MOVE`. `p_destino` es una variable local sin inicializar, con `robconf = [0,0,0,0]` y `extax = 0`, no `9E9`. | Con `GRASP`, `MoveL` puede fallar por configuración o ejes externos, o llegar con una configuración distinta a la esperada. | Clonar `CRobT` también para `GRASP` (añadirlo a la condición de la línea 52). |
| R3 | `robot/rapid/comm.mod:53,60` | `MOVE_POSE` hereda `robconf` de la pose **actual**. | Un cambio grande de orientación da error de configuración en `MoveL`. | Usar `ConfL \Off` antes del `MoveL` o calcular `robconf` del destino. Evaluarlo con cuidado. |
| R4 | `robot/rapid/comm.mod:52` | La condición clona para `cmd = "MOVE"`, pero no hay ninguna rama para `MOVE`. | `MOVE,...` responde `ERR_CMD_NO_RECONOCIDO`. Es código muerto, pero confunde. | Quitar `"MOVE"` de la condición o implementar el comando. |
| R5 | `robot/rapid/comm.mod:67-69` | `GRASP,x,y,z,?,ang`: el 4.º valor se parsea pero no se usa. La orientación es `OrientZYX(ang, 0, 180)` (`Rx(180)`), mientras que el pick & place usa `ry=180` (`Ry(180)`). | Para el mismo `ang`, `GRASP` y `MOVE_POSE` dan orientaciones que difieren 180° en z. El protocolo confunde. | Documentar o definir el 4.º campo (¿ancho de agarre?) y unificar la convención de orientación. |
| R6 | `robot/client/rapid_client.py` | `GRASP` **no tiene método en el cliente**. (`get_pose()` sí existe desde la rama `feature/get-pose`.) | No se puede usar desde Python sin `send_command` crudo. | Añadir `grasp(x, y, z, width, ang)` cuando se haga la integración (fuera de alcance aquí). |
| R7 | `robot/rapid/comm.mod:38`, `rapid_client.py` | El protocolo no tiene delimitador y RAPID recibe en un `string` de **máx. 80 caracteres**. | Mensajes largos se cortan; si se mandan dos comandos sin esperar respuesta, pueden llegar juntos. | Mantener mensajes cortos y el patrón request/response. `tests/test_rapid_client.py` comprueba que el pick & place cabe. |
| R8 | `robot/rapid/points.mod:1-7` | No tiene `MODULE ... ENDMODULE`. | No se puede cargar tal cual como módulo en RobotStudio. Sirve solo como referencia. | Envolverlo en `MODULE Puntos ... ENDMODULE` si se quiere cargar. |
| R9 | `robot/rapid/points.mod:7` vs. `experiments/02_robot_pick_place/run.py` | El `cube` de `points.mod` es `[322.51, 61.51, 62.77]`, pero el script usa `cubo_pick = [323.38, 58.88, 38.31]`. | — | **DECIDIDO por el usuario (2026-11-17):** la pose válida es la del script; `points.mod` no es fuente de verdad para el cubo y se ignora. No se modifica. |
| R10 | `robot/rapid/test.mod:40-41` | `GET_POSE` de `test.mod` usa `tool0` (no `tVision`) y devuelve solo `x,y,z`, sin cuaternión. | No servía para calibración mano-ojo. | **RESUELTO en `comm.mod`** (rama `feature/get-pose`): `GET_POSE` → `POSE,x,y,z,q1,q2,q3,q4` de `tVision` en `wobj0`. El de `test.mod` sigue igual. |
| R11 | `robot/rapid/comm.mod` y `test.mod` | Los dos definen `main` y usan el puerto 55555. | No se pueden cargar a la vez. | Cargar solo uno (documentado en `robot/rapid/README.md`). |
| R12 | `experiments/01_robot_ping/test.py:10-11, 35` | Si `socket.socket()` falla antes de asignar `s`, el `finally` lanza `NameError`. Si la conexión es rechazada, `s.close()` sí funciona. | Menor. | Inicializar `s = None` y comprobarlo en el `finally`. |

## Red neuronal (fork GR-ConvNet)

| # | Archivo:línea | Problema | Efecto | Arreglo sugerido |
|---|---|---|---|---|
| N1 | `NeuralModel/robotic-grasping/run_affordance_generator.py:34` | `INPUT_SIZE = 224`, pero el checkpoint por defecto (`cornell_rgb`, `logs/260831_1231_training_cornell_rgb`) se entrenó con **`input_size = 300`** (ver su `commandline_args.json`). | La red es totalmente convolucional y probablemente corra, pero con una escala distinta a la del entrenamiento: predicciones peores. | Usar 300 con ese checkpoint o leer `input_size`/`use_depth`/`use_rgb` del `commandline_args.json`. |
| N2 | `NeuralModel/robotic-grasping/visualize_grasp_full.py:52,62,69` | Por defecto usa `cornell-rgbd` (preentrenado, input 224, RGB-D) con `USE_DEPTH=1`. Si se cambia a `cornell_rgb` hay que ajustar a mano `USE_DEPTH` e `INPUT_SIZE`. | Error de canales (`expected 4 channels, got 3`) si no coinciden. | Igual que N1. |
| N3 | `NeuralModel/robotic-grasping/tests/test_camera_factory.py` | Importa `hardware.camera_factory`, que no existe. | El test fallaba. Ahora se **omite** (skip a nivel de módulo, con la razón). | Crear `hardware/camera_factory.py` con `normalize_camera_type`/`resolve_camera_class`, o borrar el test. |
| N4 | `NeuralModel/robotic-grasping/inference/grasp_generator.py:21-22` | Cámara forzada a `WebCamera(device_id=0)`; la RealSense está comentada y `cam_id` se ignora. | `run_grasp_generator.py` usa la webcam aunque se le pase un id de RealSense. | Elegir la cámara por parámetro. |
| N5 | `NeuralModel/robotic-grasping/hardware/camera_web.py:66-84` | La "profundidad" de la webcam es sintética (luminancia invertida, 0-2). | No es métrica: con un modelo RGB-D las predicciones carecen de sentido físico y la Z en el robot sería falsa. | Usar solo con modelos RGB, o con una cámara de profundidad real (RealSense o ZED). |
| N6 | `NeuralModel/robotic-grasping/saved_data/camera_pose.txt` | Es la **identidad** (nunca se calibró). La carpeta `saved_data/` además está ignorada por git (`.gitignore` del fork). | `grasp_generator.py` convertiría píxeles a coordenadas del robot como si la cámara estuviera en la base. | Generarla con la calibración mano-ojo (`calibration/`), en mm, cuando haya datos reales. |
| N7 | `NeuralModel/robotic-grasping/inference/grasp_generator.py:69-93` | Trabaja en **metros** (profundidad RealSense, `- 0.04`); el robot y `calibration/` usan **mm**. El ángulo se "rota" multiplicando `R @ [0,0,ang]`, lo cual solo vale si el z de la cámara es paralelo al z del robot. | Unidades mezcladas y ángulo mal transformado si la cámara está inclinada. | Al integrar: convertir a mm y transformar la orientación del agarre como rotación, no como vector. |
| N8 | `NeuralModel/robotic-grasping/inference/grasp_generator.py:37-40`, `hardware/calibrate_camera.py:35` | Se comunica por archivos `.npy` en `~/grasp-comms`, heredado del Baxter del repo original. | No encaja con el socket de RAPID. | Es el punto natural de unión con `robot/client` (fuera de alcance aquí). |
| N9 | `NeuralModel/robotic-grasping/hardware/camera.py:29` | `config.enable_device(...)` está comentado (cambio local). | Se usa la primera RealSense que encuentre, sin importar `device_id`. | Aceptable con una sola cámara. |
| N10 | `NeuralModel/robotic-grasping/run_realtime.py:19`, `run_calibration.py:8` | Valores del repo original: `saved_data/cornell_rgbd_iou_0.96` (no existe aquí) y `cam_id=830112070066` (la RealSense del autor original). | `run_realtime.py` sin `--network` falla. | Pasar `--network` explícito. |
| N11 | `local_paths.py`: `jacquard-rgb-grconvnet3-drop0-ch32` | Esa carpeta no existe en la copia local de `trained-models/`. | Elegirla en `visualize_grasp_full.py` daría `FileNotFoundError`. | Descargarla o quitarla del diccionario. |
| N12 | `CLAUDE.md` (original) | Mencionaba como mejor checkpoint `epoch_20_iou_0.93`, pero los scripts usan `epoch_48_iou_0.92`. | Confusión al elegir el modelo. | Corregido en la sección de notas del CLAUDE.md. |

## Calibración (hallazgos del experimento 04, no son bugs del código propio)

| # | Dónde | Problema | Efecto | Qué hacer |
|---|---|---|---|---|
| C1 | `cv2.calibrateHandEye(..., CALIB_HAND_EYE_DANIILIDIS)` (OpenCV 5.1.0) | Fallos catastróficos ocasionales con ruido: ~1% de las corridas da error de rotación de ~180° y de traslación de cientos de mm, aunque el p95 sea bueno. | La media y la desviación de Daniilidis no son representativas; una calibración real podría salir absurda. | Reportar mediana/p95 y revisar siempre el **residuo de consistencia**, que detecta estos fallos sin verdad de terreno. Preferir Park/Horaud o Tsai. |
| C2 | Poses con orientación fija (solo traslación) | La traslación de `T_base_cam` no es observable con ningún método (~1200 mm de error). La rotación falla con Tsai/Park/Horaud, pero Andreff y Daniilidis la recuperan (~1-2°). | Una captura de poses sin variar la orientación invalida la calibración. | Variar la orientación del gripper entre poses (inclinaciones de decenas de grados). |

## Entorno

| # | Problema | Efecto | Arreglo sugerido |
|---|---|---|---|
| E1 | En este equipo, llamar a `...\envs\neuralgrasp\python.exe` **sin activar** el entorno hace fallar el producto de matrices de numpy (MKL, `0xc06d007f`), porque carga DLLs del entorno `base`. | Crash al ejecutar `calibration/`. | Usar siempre `conda activate neuralgrasp` o `conda run -n neuralgrasp ...`. |
| E2 | `NeuralGrasp.rsstn` estaba abierto en RobotStudio durante la reorganización. | Las estaciones se quedaron en `RobotStudio/` en lugar de moverse a `robot/`. | Moverlas con `git mv` cuando RobotStudio esté cerrado, si se quiere. |

## Rutas fijas de Linux (centralizadas, no eliminadas)

Antes había rutas `/media/ulises-calderon/SteamDisk/...` escritas a mano en `run_affordance_generator.py`, `run_grasp_generator.py` y `visualize_grasp_full.py`. Ahora salen de `NeuralModel/robotic-grasping/local_paths.py`, **con el mismo valor por defecto**, y se pueden sobrescribir de tres formas (de mayor a menor prioridad):

1. `--network <ruta>` en la línea de comandos del script.
2. Variable de entorno `GRCONVNET_NETWORK=<ruta del checkpoint>`.
3. Variable de entorno `GRCONVNET_ROOT=<carpeta robotic-grasping>`; las rutas se arman como `ROOT/<relativa>`.

En Windows (PowerShell):
```powershell
$env:GRCONVNET_ROOT = "D:\Recuperacion\Escuela\9no Semestre\Robot Vision\NeuralModel\robotic-grasping"
```
