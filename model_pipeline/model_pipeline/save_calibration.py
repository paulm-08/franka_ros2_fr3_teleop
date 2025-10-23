import numpy as np
import json
import pyrealsense2 as rs
import logging
from pathlib import Path
from transforms3d.quaternions import quat2mat

from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def main():
    """
    A one-time utility to extract camera calibration from your hardware and
    hardcoded values, saving them to files for the ML pipeline.
    """
    logging.info("Starting calibration save utility...")
    
    # Define the output directory
    output_dir = paths.CONFIG_DIR / "calibration"
    output_dir.mkdir(parents=True, exist_ok=True)
    logging.info(f"Will save calibration files to: {output_dir}")

    # --- 1. Save Camera Intrinsics ---
    # This logic is adapted directly from your fr3_leap_recorder.py
    camera_serials = {
        'camera1': '151422254571',
        'camera2': '036522072607',
    }
    
    try:
        # --- Camera 1 Intrinsics ---
        pipeline1 = rs.pipeline()
        config1 = rs.config()
        config1.enable_device(camera_serials['camera1'])
        config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile1 = pipeline1.start(config1)
        intrinsics1 = profile1.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        intr_data1 = {
            "fx": intrinsics1.fx, "fy": intrinsics1.fy,
            "ppx": intrinsics1.ppx, "ppy": intrinsics1.ppy,
            "width": intrinsics1.width, "height": intrinsics1.height
        }
        out_path1 = output_dir / "intrinsics_cam1.json"
        with open(out_path1, 'w') as f:
            json.dump(intr_data1, f, indent=4)
        logging.info(f"✅ Saved Camera 1 intrinsics to {out_path1}")
        pipeline1.stop()

        # --- Camera 2 Intrinsics ---
        pipeline2 = rs.pipeline()
        config2 = rs.config()
        config2.enable_device(camera_serials['camera2'])
        config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile2 = pipeline2.start(config2)
        intrinsics2 = profile2.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        intr_data2 = {
            "fx": intrinsics2.fx, "fy": intrinsics2.fy,
            "ppx": intrinsics2.ppx, "ppy": intrinsics2.ppy,
            "width": intrinsics2.width, "height": intrinsics2.height
        }
        out_path2 = output_dir / "intrinsics_cam2.json"
        with open(out_path2, 'w') as f:
            json.dump(intr_data2, f, indent=4)
        logging.info(f"✅ Saved Camera 2 intrinsics to {out_path2}")
        pipeline2.stop()

    except Exception as e:
        logging.error(f"❌ Failed to get intrinsics from RealSense cameras. Are they plugged in?")
        logging.error(f"   Error: {e}")
        return

    # --- 2. Save Camera Extrinsics ---
    # These values are copied directly from your fr3_leap_recorder.py
    
    # Camera 1 (Front)
    cam_front_translation = [1.0322713516713722, 0.006353275596612362, 0.8234645537660135]
    cam_front_quaternion = [0.17954778476421387, -0.6799202218980005, -0.690300391513334, 0.17016596109967214] # [w, x, y, z]
    cam_front_rotation_matrix = quat2mat(cam_front_quaternion)
    
    T_base_cam1 = np.eye(4)
    T_base_cam1[:3, :3] = cam_front_rotation_matrix
    T_base_cam1[:3, 3] = cam_front_translation
    
    ext_path1 = output_dir / "T_base_cam1.npy"
    np.save(ext_path1, T_base_cam1)
    logging.info(f"✅ Saved Camera 1 extrinsics (T_base_cam1) to {ext_path1}")

    # Camera 2 (Side)
    cam_side_translation = [0.6254128691870716, -0.6593972515411284, 0.2526695484482261]
    cam_side_quaternion = [0.49543508074736786, -0.8686243685292566, 0.0059154078827518, 0.0008916902936986869] # [w, x, y, z]
    cam_side_rotation_matrix = quat2mat(cam_side_quaternion)
    
    T_base_cam2 = np.eye(4)
    T_base_cam2[:3, :3] = cam_side_rotation_matrix
    T_base_cam2[:3, 3] = cam_side_translation
    
    ext_path2 = output_dir / "T_base_cam2.npy"
    np.save(ext_path2, T_base_cam2)
    logging.info(f"✅ Saved Camera 2 extrinsics (T_base_cam2) to {ext_path2}")

    logging.info("\nCalibration file generation complete. Your config.yaml is now pointing to valid files.")

if __name__ == "__main__":
    main()
