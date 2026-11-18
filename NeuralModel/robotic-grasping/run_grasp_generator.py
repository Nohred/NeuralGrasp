from inference.grasp_generator import GraspGenerator
from local_paths import model_path, resolve_network

if __name__ == '__main__':
    generator = GraspGenerator(
        cam_id=830112070066,
        # Ruta centralizada en local_paths.py (sobrescribible con --network,
        # GRCONVNET_NETWORK o GRCONVNET_ROOT).
        saved_model_path=resolve_network(model_path("jacquard-rgbd-grconvnet3-drop0-ch32")),
        visualize=True
    )
    generator.load_model()
    generator.run()

