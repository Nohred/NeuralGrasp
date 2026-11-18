"""Rutas locales de checkpoints, centralizadas en un solo lugar.

Archivo añadido en este proyecto (no existe en skumra/robotic-grasping).

Antes, run_affordance_generator.py, run_grasp_generator.py y visualize_grasp_full.py
tenían rutas absolutas de Linux escritas a mano. Ahora todas salen de aquí, con el
MISMO valor por defecto que tenían, y se pueden sobrescribir sin editar código:

  1. Variable de entorno ``GRCONVNET_ROOT``: carpeta raíz de ``robotic-grasping``
     (p. ej. ``D:\\Recuperacion\\Escuela\\9no Semestre\\Robot Vision\\NeuralModel\\robotic-grasping``).
  2. Variable de entorno ``GRCONVNET_NETWORK``: ruta completa de un checkpoint; tiene
     prioridad sobre el valor por defecto de cada script.
  3. Argumento ``--network <ruta>`` en cada script; tiene prioridad sobre todo lo anterior.
"""
import argparse
import os

# Valor por defecto: el que estaba fijo en los scripts (equipo Linux del autor).
DEFAULT_ROOT = "/media/ulises-calderon/SteamDisk/Recuperacion/Escuela/9no Semestre/Robot Vision/NeuralModel/robotic-grasping"

ROOT = os.environ.get("GRCONVNET_ROOT", DEFAULT_ROOT)

# Checkpoints relativos a ROOT. Los pesos NO se versionan en git (ver NeuralModel/README.md).
MODEL_RELPATHS = {
    # Entrenado localmente: Cornell, solo RGB, input_size 300.
    "cornell_rgb": "logs/260831_1231_training_cornell_rgb/epoch_48_iou_0.92",
    # Preentrenados del repo original (trained-models/).
    "cornell-rgbd": "trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98",
    "jacquard-rgbd-grconvnet3-drop0-ch32": "trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93",
    # OJO: esta carpeta no existe en la copia local de trained-models/.
    "jacquard-rgb-grconvnet3-drop0-ch32": "trained-models/jacquard-rgb-grconvnet3-drop0-ch32/epoch_48_iou_0.92",
}


def model_path(key):
    """Ruta absoluta del checkpoint ``key`` bajo ROOT (con '/' igual que antes)."""
    return ROOT.rstrip("/\\") + "/" + MODEL_RELPATHS[key]


def resolve_network(default_path):
    """Aplica las sobrescrituras: --network > GRCONVNET_NETWORK > default_path.

    Usa ``parse_known_args`` para no romper scripts que no definen otros argumentos.
    """
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--network", type=str, default=None)
    args, _ = parser.parse_known_args()
    if args.network:
        return args.network
    return os.environ.get("GRCONVNET_NETWORK", default_path)
