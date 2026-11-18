import logging
from types import SimpleNamespace

import cv2
import matplotlib.pyplot as plt
import numpy as np

logger = logging.getLogger(__name__)


class WebCamera:
    """
    Wrapper around a standard webcam that mimics the RealSenseCamera API.

    The repository expects a camera object with:
      - connect()
      - get_image_bundle()
      - intrinsics
      - scale

    Since a webcam has no real depth sensor, we synthesize a depth map from the
    grayscale intensity of the RGB image. This keeps the rest of the pipeline
    compatible while allowing the model to receive a depth-like tensor.
    """

    def __init__(self, device_id=0, width=640, height=480, fps=30):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.scale = 1.0
        self.intrinsics = SimpleNamespace(
            fx=float(width),
            fy=float(height),
            ppx=float(width / 2.0),
            ppy=float(height / 2.0),
        )

    def connect(self):
        """Open the webcam and configure it to the desired resolution/fps."""
        self.cap = cv2.VideoCapture(int(self.device_id))
        if not self.cap.isOpened():
            raise RuntimeError(f'Could not open webcam device {self.device_id}')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Some webcams ignore requested values; read the actual frame size.
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_w > 0 and actual_h > 0:
            self.width = actual_w
            self.height = actual_h
            self.intrinsics = SimpleNamespace(
                fx=float(self.width),
                fy=float(self.height),
                ppx=float(self.width / 2.0),
                ppy=float(self.height / 2.0),
            )

        logger.info('Web camera connected: %dx%d @ %dfps', self.width, self.height, self.fps)

    def _synthetic_depth_from_rgb(self, rgb_image):
        """
        Create a depth-like image from the luminance of the RGB frame.

        This is not a true depth sensor. It is only a placeholder to keep the
        rest of the code compatible with the RealSense pipeline.
        """
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY).astype(np.float32)
        gray = gray / 255.0

        # Brighter pixels become closer; darker pixels become farther.
        # This produces a soft depth map with a valid range between 0 and 1.
        depth = 1.0 - gray
        depth = np.clip(depth, 0.0, 1.0)

        # Scale depth to a more realistic metric-like range for the model.
        depth = depth * 2.0
        depth = np.expand_dims(depth, axis=2)
        return depth.astype(np.float32)

    def get_image_bundle(self):
        """Return a dictionary with RGB and a depth-like image."""
        if self.cap is None:
            raise RuntimeError('Camera is not connected. Call connect() first.')

        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError('Failed to read frame from webcam.')

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        depth = self._synthetic_depth_from_rgb(rgb)

        return {
            'rgb': rgb,
            'aligned_depth': depth,
        }

    def plot_image_bundle(self):
        images = self.get_image_bundle()
        rgb = images['rgb']
        depth = images['aligned_depth']

        fig, ax = plt.subplots(1, 2, squeeze=False)
        ax[0, 0].imshow(rgb)
        ax[0, 1].imshow(depth.squeeze(axis=2), cmap=plt.cm.gray)
        ax[0, 0].set_title('rgb')
        ax[0, 1].set_title('synthetic_depth')
        plt.show()


# Backwards-compatible alias to keep naming intuitive.
CameraWeb = WebCamera
