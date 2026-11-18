# NeuralModel — fork de GR-ConvNet (robotic-grasping)

## Qué es

`robotic-grasping/` es una copia de [skumra/robotic-grasping](https://github.com/skumra/robotic-grasping) (GR-ConvNet, Kumra et al., paper en `docs/references/1909.04810v4.pdf`), que predice agarres antipodales a partir de imágenes RGB/RGB-D.

- **Commit de origen:** `183c6f68c44c1c7ff0f07707e2db6fcfd6840d2d` ("Updated path for Cornell dataset").
- Su `.git` original se movió **fuera del repo**, a `D:\Recuperacion\Escuela\9no Semestre\RobotVision_fuera_del_repo\robotic-grasping.git`, para que estos archivos se versionen en NeuralGrasp. No se reestructuró nada por dentro.
- Las diferencias exactas con el original, al momento de importarlo, están en [`local_changes_vs_upstream.patch`](local_changes_vs_upstream.patch).

### Qué es del repo original y qué se añadió aquí

| Archivo | Origen | Notas |
|---|---|---|
| `train_network.py`, `evaluate.py`, `run_offline.py`, `run_realtime.py`, `run_calibration.py`, `inference/models/*`, `utils/*`, `hardware/device.py`, `hardware/calibrate_camera.py` | **original** | sin cambios |
| `hardware/camera.py` (RealSense, `pyrealsense2`) | original **modificado** | `config.enable_device(...)` comentado |
| `inference/grasp_generator.py` | original **modificado** | cámara forzada a `WebCamera` (dummy para probar sin RealSense) |
| `run_grasp_generator.py` | original **modificado** | ruta del checkpoint (ahora vía `local_paths.py`) |
| `README.md` del fork | original **modificado** | ejemplo de entrenamiento RGB añadido |
| `hardware/camera_web.py` | **añadido** | webcam con la misma API que la RealSense; profundidad **sintética** (dummy, no métrica) |
| `run_affordance_generator.py` | **añadido** | inferencia en vivo + overlay OpenCV (RealSense por defecto) |
| `visualize_grasp_full.py` | **añadido** | visor en vivo con mapas Q/ángulo/ancho (matplotlib) |
| `tests/test_camera_factory.py` | **añadido** | test de un módulo que **no existe**; ahora se omite (skip) |
| `local_paths.py` | **añadido en esta reorganización** | rutas de checkpoints centralizadas |
| `logs/260831_*_training_cornell_rgb/` | **añadido** | entrenamientos locales (Cornell, solo RGB, input 300) |

### Dónde están los pesos y datos (no versionados)

Están ignorados por git (`.gitignore` de la raíz) y siguen en disco, sin moverse:

| Qué | Ruta local | Cómo recuperarlo si falta |
|---|---|---|
| Dataset Cornell | `robotic-grasping/data/archive/` (carpetas `01`–`10`, `backgrounds`) | `utils/get_cornell.sh` o Kaggle |
| Checkpoints entrenados localmente | `robotic-grasping/logs/260831_1231_training_cornell_rgb/epoch_*_iou_*` (mejor IoU: `epoch_20_iou_0.93`; los scripts usan `epoch_48_iou_0.92`) | volver a entrenar (ver abajo) |
| Checkpoints preentrenados | `robotic-grasping/trained-models/*/epoch_*_iou_*` | vienen en el repo original (`git clone skumra/robotic-grasping`) |
| Eventos de TensorBoard | `robotic-grasping/logs/*/events.out.tfevents.*` | — |

Sí se versionan los `arch.txt` y `commandline_args.json`, que dicen con qué `input_size`, `use_depth` y `use_rgb` se entrenó cada checkpoint.

## Requisitos

- Python con las dependencias del fork: `robotic-grasping/requirements.txt` (torch, torchvision, opencv-python, scikit-image, pyrealsense2, …). **No** están instaladas en el entorno conda `neuralgrasp`, que solo tiene lo necesario para `calibration/`.
- GPU con CUDA recomendada para entrenar.
- Cámara: Intel RealSense (`pyrealsense2`) o webcam (`camera_web.py`).

## Cómo ejecutar

Todo **desde dentro de `robotic-grasping/`** (los imports son relativos a esa carpeta). Sugerencia: un entorno conda aparte (p. ej. `grconvnet`), para no mezclar torch con `neuralgrasp`.

```powershell
cd NeuralModel/robotic-grasping
pip install -r requirements.txt

# Rutas de checkpoints en Windows (el valor por defecto es la ruta de Linux original)
$env:GRCONVNET_ROOT = "D:\Recuperacion\Escuela\9no Semestre\Robot Vision\NeuralModel\robotic-grasping"

# Entrenar (lo que se usó para logs/260831_1231_training_cornell_rgb)
python train_network.py --dataset cornell --dataset-path data/archive --network grconvnet3 --use-depth 0 --use-rgb 1 --input-size 300 --description training_cornell_rgb

# Evaluar
python evaluate.py --network logs/260831_1231_training_cornell_rgb/epoch_20_iou_0.93 --dataset cornell --dataset-path data/archive --use-depth 0 --use-rgb 1 --input-size 300 --iou-eval

# Inferencia en vivo (ver experiments/03_grasp_net_live)
python run_affordance_generator.py --network logs/260831_1231_training_cornell_rgb/epoch_48_iou_0.92
```

## Qué esperar

- Entrenamiento: carpetas nuevas en `logs/` con `epoch_XX_iou_Y`.
- Inferencia en vivo: una ventana con el mejor agarre dibujado (rectángulo verde y centro rojo).

**Estado:** el entrenamiento y la inferencia los hizo el usuario en Linux (los `logs/` lo demuestran: probado por el usuario, según CLAUDE.md). En esta reorganización **no se ejecutó nada** del fork (sin GPU, cámara ni torch en el entorno). Solo se comprobó que los scripts editados compilan (`py_compile`), que `local_paths.py` devuelve exactamente las rutas de antes y que `tests/test_camera_factory.py` queda como *skipped*.

## Limitaciones y problemas conocidos

Ver [docs/known_issues.md](../docs/known_issues.md), sección "Red neuronal" (N1-N12). Lo principal: `INPUT_SIZE = 224` con un checkpoint entrenado a 300 (N1), profundidad sintética de la webcam (N5), `camera_pose.txt` sin calibrar (N6) y unidades en metros (N7).

## Siguiente paso

Leer `input_size`/`use_depth`/`use_rgb` del `commandline_args.json` del checkpoint en lugar de fijarlos a mano. Después, cuando toque integrar, convertir la salida de `grasp_generator.py` a mm usando la `T_base_cam` de `calibration/`.
