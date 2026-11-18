# 03 — Predicción de agarres en vivo con GR-ConvNet (sin robot)

## Qué es

Inferencia en tiempo real de la red sobre la imagen de una cámara, con los agarres dibujados encima. No mueve el robot. No hay código propio en esta carpeta: los scripts viven en el fork (`NeuralModel/robotic-grasping/`) y se ejecutan desde ahí, porque sus imports son relativos a esa carpeta y el fork no se reestructuró.

| Script | Qué muestra | Cámara por defecto | Checkpoint por defecto |
|---|---|---|---|
| `run_affordance_generator.py` (añadido) | Ventana OpenCV con el mejor agarre | `CAMERA_TYPE = 'realsense'` | `cornell_rgb` (local, RGB, **input 300**) con `INPUT_SIZE = 224` ⚠ |
| `visualize_grasp_full.py` (añadido) | Ventana matplotlib: RGB + agarres, profundidad, mapas Q, ángulo y ancho | `'realsense'` | `cornell-rgbd` (preentrenado, RGB-D, input 224) |
| `run_realtime.py` (original) | Visualización del repo original | RealSense | requiere `--network` |

**Sobre la RealSense:** el código está en `hardware/camera.py` (del repo original, con `pyrealsense2`; `enable_device` comentado). `run_affordance_generator.py` y `visualize_grasp_full.py` la usan por defecto. Al final de `run_affordance_generator.py` hay, comentado, un snippet mínimo de `pyrealsense2` para ver el stream de color. Se asume que esos scripts son la prueba de RealSense del usuario.

## Requisitos

- Dependencias del fork (`NeuralModel/robotic-grasping/requirements.txt`: torch, opencv, pyrealsense2, …).
- Una Intel RealSense conectada, o una webcam cambiando `CAMERA_TYPE = 'web'` en el script.
- GPU opcional (`USE_CPU = True` para forzar CPU).
- Los checkpoints en disco (no están en git; ver `NeuralModel/README.md`).

## Cómo ejecutar

```powershell
cd NeuralModel/robotic-grasping
$env:GRCONVNET_ROOT = "D:\Recuperacion\Escuela\9no Semestre\Robot Vision\NeuralModel\robotic-grasping"

python run_affordance_generator.py                    # checkpoint por defecto (cornell_rgb)
python run_affordance_generator.py --network trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93
python visualize_grasp_full.py                        # cornell-rgbd por defecto
```

Si cambias de checkpoint, **ajusta `USE_DEPTH`, `USE_RGB` e `INPUT_SIZE`** en el script para que coincidan con su `commandline_args.json` o `arch.txt`. Si no, aparece el error de canales.

Pulsa `q` para salir.

## Qué esperar

Una ventana con la imagen recortada al centro y uno o dos rectángulos de agarre (verde) con su centro (rojo).

**Estado:** probado por el usuario en Linux (según CLAUDE.md). **Sin verificar** en Windows ni tras centralizar las rutas: no hay cámara, GPU ni torch en el entorno de Claude Code. Solo se comprobó que los scripts compilan y que las rutas por defecto son idénticas a las anteriores.

## Limitaciones y problemas conocidos

- `INPUT_SIZE = 224` con un checkpoint entrenado a 300 (N1).
- Con webcam, la profundidad es sintética y no métrica (N5).
- Ver `docs/known_issues.md`, N1-N11.

## Siguiente paso

Leer la configuración del checkpoint de su `commandline_args.json`, y después probar con la ZED en la Jetson (manual en `docs/references/`; fuera de alcance aquí).
