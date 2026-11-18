import logging
import os

import cv2
import numpy as np
import torch

from hardware.camera import RealSenseCamera
from hardware.camera_web import WebCamera
from hardware.device import get_device
from inference.post_process import post_process_output
from local_paths import model_path, resolve_network
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps

logging.basicConfig(level=logging.INFO)

# Path to the trained model checkpoint (rutas centralizadas en local_paths.py;
# sobrescribible con --network, GRCONVNET_NETWORK o GRCONVNET_ROOT).
# Only RGB
NETWORK_PATH = resolve_network(model_path("cornell_rgb"))
# RGB-D Jacquard
# NETWORK_PATH = resolve_network(model_path("jacquard-rgbd-grconvnet3-drop0-ch32"))

# Camera backend to use: 'web' or 'realsense'.
CAMERA_TYPE = 'realsense'
# Webcam index or Realsense device id.
CAMERA_ID = 0
# If 1, include depth in the model input; otherwise, only RGB.
USE_DEPTH = 0
# If 1, include RGB in the model input; otherwise, only depth.
USE_RGB = 1
# Crop/output size used by the model.
INPUT_SIZE = 224
# If True, force CPU execution instead of GPU.
USE_CPU = False
# Number of detected grasps to overlay on the image.
N_GRASPS = 1
# Name of the OpenCV window.
WINDOW_NAME = 'Affordance Detection'


def make_camera(camera_type, device_id):
    if camera_type == 'web':
        cam = WebCamera(device_id=device_id, width=640, height=480, fps=30)
    else:
        cam = RealSenseCamera(device_id=device_id, width=640, height=480, fps=6)
    cam.connect()
    return cam


def draw_grasps(frame, grasps):
    out = frame.copy()
    for g in grasps:
        pts = np.asarray(g.as_gr.points, dtype=np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        cx = int(round(g.center[1]))
        cy = int(round(g.center[0]))
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
    return out


def main():
    if not os.path.exists(NETWORK_PATH):
        raise FileNotFoundError('Model not found: {}'.format(NETWORK_PATH))

    cam = make_camera(CAMERA_TYPE, CAMERA_ID)
    cam_data = CameraData(output_size=INPUT_SIZE, include_depth=bool(USE_DEPTH), include_rgb=bool(USE_RGB))


    device = get_device(USE_CPU)
    print('Loading model...')
    net = torch.load(NETWORK_PATH)
    net.to(device)
    net.eval()
    print('Model loaded.')

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    while True:
        try:
            image_bundle = cam.get_image_bundle()
            rgb = image_bundle['rgb']
            depth = image_bundle['aligned_depth']

            x, _, _ = cam_data.get_data(rgb=rgb, depth=depth)
            with torch.no_grad():
                xc = x.to(device)
                pred = net.predict(xc)

            q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
            grasps = detect_grasps(q_img, ang_img, width_img=width_img, no_grasps=N_GRASPS)

            rgb_vis = cam_data.get_rgb(rgb, False)
            # rgb_vis = np.transpose(rgb_vis, (1, 2, 0))
            rgb_vis = np.ascontiguousarray(rgb_vis)

            if rgb_vis.shape[0] != rgb.shape[0] or rgb_vis.shape[1] != rgb.shape[1]:
                rgb_vis = cv2.resize(rgb_vis, (rgb.shape[1], rgb.shape[0]))

            overlay = draw_grasps(rgb_vis, grasps)
            cv2.imshow(WINDOW_NAME, cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        except KeyboardInterrupt:
            break
        except Exception as exc:
            logging.exception('Inference loop failed: %s', exc)
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()



    # import pyrealsense2 as rs
    # import numpy as np
    # import cv2

    # # Configure color stream
    # pipeline = rs.pipeline()
    # config = rs.config()
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # # Start streaming
    # pipeline.start(config)

    # try:
    #     while True:
    #         # Wait for a coherent pair of frames
    #         frames = pipeline.wait_for_frames()
    #         color_frame = frames.get_color_frame()
    #         if not color_frame:
    #             continue

    #         # Convert images to numpy arrays
    #         color_image = np.asanyarray(color_frame.get_data())

    #         # Display RGB window using OpenCV
    #         cv2.imshow('RealSense RGB', color_image)

    #         # Press 'q' to close the window
    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break

    # finally:
    #     # Stop streaming
    #     pipeline.stop()
    #     cv2.destroyAllWindows()

