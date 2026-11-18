"""
Visualizador en tiempo real para el modelo GR-ConvNet de robotic-grasping
(skumra/robotic-grasping). Muestra en una sola ventana TODO lo que el
modelo produce, no solo las cajas (grasp rectangles):

  - RGB con los N mejores grasps dibujados encima
  - Depth recortado (si se usa)
  - Mapa de calidad Q (dónde "cree" el modelo que se puede agarrar)
  - Mapa de ángulo (orientación del gripper, en grados)
  - Mapa de ancho de apertura del gripper (en píxeles)
  - Panel de texto con los valores numéricos de cada grasp detectado

Todos los parámetros configurables están en la sección "CONFIGURACIÓN"
justo debajo de los imports. Edítalos ahí, no hace falta pasar
argumentos por línea de comandos.
"""

import sys

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn

from hardware.camera import RealSenseCamera
from hardware.camera_web import WebCamera
from hardware.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps


# ============================== CONFIGURACIÓN ==============================
# Ruta al checkpoint entrenado (el archivo que guardó torch.save del modelo).
# IMPORTANTE: debe coincidir con USE_DEPTH/USE_RGB de abajo. Un checkpoint
# entrenado con RGB-D (4 canales) NO se puede correr con USE_DEPTH=0,
# USE_RGB=1 (3 canales) y viceversa: eso es justo el error
# "expected input[...] to have 4 channels, but got 3 channels instead".
path_dict = {
    "jacquard-rgbd-grconvnet3-drop0-ch32": "/media/ulises-calderon/SteamDisk/Recuperacion/Escuela/9no Semestre/Robot Vision/NeuralModel/robotic-grasping/trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93",
    "jacquard-rgb-grconvnet3-drop0-ch32": "/media/ulises-calderon/SteamDisk/Recuperacion/Escuela/9no Semestre/Robot Vision/NeuralModel/robotic-grasping/trained-models/jacquard-rgb-grconvnet3-drop0-ch32/epoch_48_iou_0.92",
    "cornell_rgb" : '/media/ulises-calderon/SteamDisk/Recuperacion/Escuela/9no Semestre/Robot Vision/NeuralModel/robotic-grasping/logs/260831_1231_training_cornell_rgb/epoch_48_iou_0.92',
    "cornell-rgbd":"/media/ulises-calderon/SteamDisk/Recuperacion/Escuela/9no Semestre/Robot Vision/NeuralModel/robotic-grasping/trained-models/cornell-randsplit-rgbd-grconvnet3-drop1-ch32/epoch_19_iou_0.98"
}




NETWORK_PATH = path_dict["cornell-rgbd"]  # Cambiar a otro checkpoint si se quiere.
# Backend de cámara: 'web' (webcam vía OpenCV) o 'realsense' (Intel RealSense).
CAMERA_TYPE = 'realsense'
# CAMERA_TYPE = 'web'  # Cambiar a 'web' si se quiere usar una webcam normal.

# Índice de la webcam o ID del dispositivo RealSense.
CAMERA_ID = 0

# 1 si el checkpoint fue entrenado incluyendo el canal de profundidad, 0 si no.
# Para el modelo "jacquard-rgbd..." debe ser 1.
USE_DEPTH = 1

# 1 si el checkpoint fue entrenado incluyendo canales RGB, 0 si no.
# Para el modelo "jacquard-rgbd..." debe ser 1 (RGB-D = RGB + Depth = 4 canales).
USE_RGB = 1

# Tamaño del crop cuadrado (en píxeles) que espera la red como entrada.
INPUT_SIZE = 224

# Cuántos grasps detectar y dibujar por frame.
N_GRASPS = 2

# True para forzar CPU aunque haya GPU disponible.
USE_CPU = False
# ============================================================================


def make_camera(camera_type, device_id):
    if camera_type == 'web':
        cam = WebCamera(device_id=device_id, width=640, height=480, fps=30)
    else:
        cam = RealSenseCamera(device_id=device_id, width=640, height=480, fps=6)
    cam.connect()
    return cam


def to_2d(img):
    """Devuelve una vista 2D (H, W) de un array que puede venir como (H,W), (H,W,1) o (1,H,W)."""
    arr = np.asarray(img)
    arr = np.squeeze(arr)
    return arr


def expected_input_channels(use_rgb, use_depth):
    return (3 if use_rgb else 0) + (1 if use_depth else 0)


def find_first_conv_in_channels(net):
    """Recorre el modelo y devuelve el in_channels de la primera Conv2d que encuentre."""
    for module in net.modules():
        if isinstance(module, nn.Conv2d):
            return module.in_channels
    return None


def check_channel_config(net, use_rgb, use_depth):
    """Valida que USE_RGB/USE_DEPTH coincidan con lo que espera el checkpoint
    ANTES de correr la cámara, para no fallar a mitad del loop con un error
    críptico de PyTorch."""
    expected = expected_input_channels(use_rgb, use_depth)
    model_channels = find_first_conv_in_channels(net)

    if model_channels is None:
        print('Aviso: no se pudo inspeccionar la primera capa Conv2d del modelo; '
              'se omite la validación automática de canales.')
        return

    if model_channels != expected:
        raise RuntimeError(
            "Config de canales inconsistente con el checkpoint.\n"
            "  El checkpoint '{path}' espera {model_channels} canal(es) de entrada.\n"
            "  Con USE_RGB={use_rgb} y USE_DEPTH={use_depth} se estan generando {expected} canal(es).\n"
            "  Ajusta USE_RGB/USE_DEPTH en la seccion CONFIGURACION para que coincidan "
            "con como se entreno ese checkpoint (3 = solo RGB, 1 = solo depth, 4 = RGB-D)."
            .format(path=NETWORK_PATH, model_channels=model_channels,
                    use_rgb=use_rgb, use_depth=use_depth, expected=expected)
        )


class LiveGraspViewer:
    def __init__(self):
        self.device = get_device(USE_CPU)

        print('Cargando modelo desde: {}'.format(NETWORK_PATH))
        self.net = torch.load(NETWORK_PATH, map_location=self.device)
        self.net.to(self.device)
        self.net.eval()
        print('Modelo cargado en:', self.device)

        check_channel_config(self.net, USE_RGB, USE_DEPTH)

        self.cam = make_camera(CAMERA_TYPE, CAMERA_ID)
        self.cam_data = CameraData(
            output_size=INPUT_SIZE,
            include_depth=bool(USE_DEPTH),
            include_rgb=bool(USE_RGB),
        )

        self.closed = False
        self._build_figure()

    # ---------------------------------------------------------- figura ----
    def _build_figure(self):
        self.fig, self.axes = plt.subplots(2, 3, figsize=(15, 9))
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.fig.canvas.mpl_connect('close_event', self._on_close)

        blank = np.zeros((INPUT_SIZE, INPUT_SIZE))
        blank_rgb = np.zeros((INPUT_SIZE, INPUT_SIZE, 3))

        ax_rgb, ax_depth, ax_q = self.axes[0]
        ax_ang, ax_width, ax_txt = self.axes[1]

        self.im_rgb = ax_rgb.imshow(blank_rgb)
        ax_rgb.set_title('RGB + grasps detectados')
        ax_rgb.axis('off')
        self.grasp_lines = []   # artistas de las cajas, se limpian cada frame
        self.grasp_markers = []

        if USE_DEPTH:
            self.im_depth = ax_depth.imshow(blank, cmap='gray')
            ax_depth.set_title('Depth (recortado)')
        else:
            self.im_depth = None
            ax_depth.text(0.5, 0.5, 'Depth no usado\n(USE_DEPTH = 0)',
                           ha='center', va='center')
        ax_depth.axis('off')

        self.im_q = ax_q.imshow(blank, cmap='jet', vmin=0, vmax=1)
        ax_q.set_title('Q: calidad de agarre')
        ax_q.axis('off')
        self.fig.colorbar(self.im_q, ax=ax_q, fraction=0.046)

        self.im_ang = ax_ang.imshow(blank, cmap='hsv', vmin=-90, vmax=90)
        ax_ang.set_title('Ángulo del gripper (grados)')
        ax_ang.axis('off')
        self.fig.colorbar(self.im_ang, ax=ax_ang, fraction=0.046)

        self.im_width = ax_width.imshow(blank, cmap='jet', vmin=0, vmax=150)
        ax_width.set_title('Apertura del gripper (px)')
        ax_width.axis('off')
        self.fig.colorbar(self.im_width, ax=ax_width, fraction=0.046)

        ax_txt.axis('off')
        self.ax_txt = ax_txt
        self.txt_artist = ax_txt.text(0.02, 0.98, '', va='top', ha='left',
                                       family='monospace', fontsize=9)

        self.ax_rgb = ax_rgb
        plt.tight_layout()
        plt.ion()
        self.fig.show()

    def _on_key(self, event):
        if event.key == 'q':
            self.closed = True

    def _on_close(self, event):
        self.closed = True

    # -------------------------------------------------------- inferencia --
    def step(self):
        image_bundle = self.cam.get_image_bundle()
        rgb = image_bundle['rgb']
        depth = image_bundle.get('aligned_depth')

        x, depth_img, rgb_img = self.cam_data.get_data(rgb=rgb, depth=depth)

        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.net.predict(xc)

        q_img, ang_img, width_img = post_process_output(
            pred['pos'], pred['cos'], pred['sin'], pred['width']
        )

        grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=N_GRASPS)

        rgb_vis = self.cam_data.get_rgb(rgb, False)
        rgb_vis = np.ascontiguousarray(rgb_vis)

        self._update_plots(rgb_vis, depth_img, q_img, ang_img, width_img, grasps)

    def _update_plots(self, rgb_vis, depth_img, q_img, ang_img, width_img, grasps):
        self.im_rgb.set_data(rgb_vis)

        if self.im_depth is not None and depth_img is not None:
            self.im_depth.set_data(to_2d(depth_img))

        self.im_q.set_data(q_img)
        self.im_ang.set_data(np.degrees(ang_img))
        self.im_width.set_data(width_img)

        for artist in self.grasp_lines + self.grasp_markers:
            artist.remove()
        self.grasp_lines = []
        self.grasp_markers = []

        colors = plt.cm.tab10(np.linspace(0, 1, max(len(grasps), 1)))
        report_lines = ['Grasps detectados: {}'.format(len(grasps)), '']
        for i, g in enumerate(grasps):
            color = colors[i % len(colors)]

            points = np.vstack((g.as_gr.points, g.as_gr.points[0]))
            line, = self.ax_rgb.plot(points[:, 1], points[:, 0], color=color, linewidth=2)
            self.grasp_lines.append(line)

            marker, = self.ax_rgb.plot(g.center[1], g.center[0], 'x', color=color, markersize=8, mew=2)
            self.grasp_markers.append(marker)

            row, col = int(g.center[0]), int(g.center[1])
            row = min(max(row, 0), q_img.shape[0] - 1)
            col = min(max(col, 0), q_img.shape[1] - 1)
            quality = float(q_img[row, col])
            angle_deg = float(np.degrees(g.angle))
            length_px = float(g.length)
            width_px = float(g.width)

            report_lines.append(
                '#{idx} centro=(fila={r}, col={c})  angulo={ang:6.1f} deg  '
                'apertura={length:5.1f}px  grosor={width:5.1f}px  Q={q:.3f}'.format(
                    idx=i, r=row, c=col, ang=angle_deg, length=length_px, width=width_px, q=quality
                )
            )

        if not grasps:
            report_lines.append('(sin detecciones por encima del umbral)')

        self.txt_artist.set_text('\n'.join(report_lines))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def run(self):
        try:
            while not self.closed and plt.fignum_exists(self.fig.number):
                self.step()
                plt.pause(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            release = getattr(self.cam, 'release', None) or getattr(self.cam, 'stop', None)
            if callable(release):
                try:
                    release()
                except Exception:
                    pass
            plt.close(self.fig)


def main():
    viewer = LiveGraspViewer()
    print("Ventana abierta. Presiona 'q' o cierra la ventana para salir.")
    viewer.run()


if __name__ == '__main__':
    sys.exit(main())
