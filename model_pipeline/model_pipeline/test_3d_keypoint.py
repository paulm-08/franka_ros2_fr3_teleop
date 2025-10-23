import cv2
import torch
import numpy as np
import logging
from pathlib import Path
import pyrealsense2 as rs
import yaml
import inquirer
import os
import pickle

# Import your pipeline components
from model_pipeline import paths
from model_pipeline.keypoint_extractor import KeypointExtractor
from model_pipeline.utils import load_frame_paths # Assuming this is in your utils

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def find_pkl_files(search_path):
    """Finds all .pkl dataset files."""
    return [p.relative_to(paths.WORKSPACE_ROOT) for p in search_path.glob("*.pkl")]

def draw_detections(image, results, model_names):
    """
    Helper function to find the most confident bounding box for each class
    and draw only those boxes on the image.
    """
    best_boxes = {} # Dictionary to store the best box for each class ID
    for box in results.boxes:
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])
        if cls_id not in best_boxes or confidence > best_boxes[cls_id]['confidence']:
            best_boxes[cls_id] = {'box': box.xyxy[0], 'confidence': confidence, 'cls_id': cls_id}

    for data in best_boxes.values():
        x1, y1, x2, y2 = map(int, data['box'])
        confidence = data['confidence']
        class_name = model_names[data['cls_id']]
        label = f'{class_name} {confidence:.2f}'
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    return image

def main():
    # --- 1. Load Configuration ---
    config_path = paths.DEFAULT_CONFIG_PATH
    if not config_path.exists():
        logging.error(f"Config file not found at: {config_path}"); return
    with open(config_path, 'r') as f: config = yaml.safe_load(f)
    
    vision_config = config.get("vision", {})
    if not vision_config.get("use_keypoint_extractor", False) or not vision_config.get("use_3d", False):
        logging.error("This script requires 'use_keypoint_extractor: true' and 'use_3d: true' in config.yaml.")
        return

    # --- 2. Interactive Mode Selection ---
    try:
        source_question = [
            inquirer.List('source', message="Select data source", choices=['Live Feed', 'Dataset File']),
        ]
        source_answer = inquirer.prompt(source_question)
        if not source_answer: return
        data_source = source_answer['source']

        dataset_frames = None
        frame_idx = 0
        
        if data_source == 'Dataset File':
            pkl_choices = find_pkl_files(paths.PROCESSED_DATA_DIR)
            if not pkl_choices: logging.error(f"No .pkl datasets found in {paths.PROCESSED_DATA_DIR}."); return
            
            dataset_question = [
                inquirer.List('dataset_pkl', message="Select the dataset (.pkl) to test on", choices=pkl_choices)
            ]
            dataset_answer = inquirer.prompt(dataset_question)
            dataset_path = paths.WORKSPACE_ROOT / dataset_answer['dataset_pkl']
            with open(dataset_path, "rb") as f: all_trajectories = pickle.load(f)
            
            # For this test, just use the first trajectory
            traj_data = all_trajectories[0]
            # We need to reconstruct the original frame paths
            # This is a bit of a hack; assumes dataset 'name' is in the config or similar
            # A simpler way is to just browse the raw demo folders.
            logging.warning("Dataset mode is complex; using raw demo folder browser instead.")
            logging.error("Dataset mode not fully implemented in this test script. Please select Live Feed or update script.")
            return # Let's stick to a live feed test for simplicity of this script
            
    except (KeyboardInterrupt, TypeError):
        logging.info("\nSelection cancelled.")
        return
        
    # --- 3. Initialize Hardware (RealSense) ---
    pipelines = {}
    aligners = {}
    camera_serials = {
        'cam1': '151422254571', # Use your 'camera1' serial
        'cam2': '036522072607', # Use your 'camera2' serial
    }
    
    try:
        for cam_id, serial in camera_serials.items():
            pipeline = rs.pipeline()
            rs_config = rs.config()
            rs_config.enable_device(serial)
            rs_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(rs_config)
            aligners[cam_id] = rs.align(rs.stream.color)
            pipelines[cam_id] = pipeline
            logging.info(f"RealSense camera {cam_id} ({serial}) started.")
    except Exception as e:
        logging.error(f"Failed to start RealSense cameras: {e}")
        return

    # --- 4. Initialize KeypointExtractors in 3D Mode ---
    try:
        extractor1 = KeypointExtractor(
            model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
            confidence_threshold=vision_config.get("confidence_threshold", 0.1),
            use_3d=True,
            intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam1")),
            extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam1")),
            device="cuda" if torch.cuda.is_available() else "cpu"
        )
        extractor2 = KeypointExtractor(
            model_path=str(paths.WORKSPACE_ROOT / vision_config["yolo_model_path"]),
            confidence_threshold=vision_config.get("confidence_threshold", 0.1),
            use_3d=True,
            intrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("intrinsics_path_cam2")),
            extrinsics_path=str(paths.WORKSPACE_ROOT / vision_config.get("extrinsics_path_cam2")),
            device="cuda" if torch.cuda.is_available() else "cpu"
        )
    except Exception as e:
        logging.error(f"Failed to initialize 3D KeypointExtractors: {e}")
        for p in pipelines.values(): p.stop()
        return

    logging.info("Extractors initialized in 3D mode. Press 'q' to quit.")
    np.set_printoptions(precision=4, suppress=True)

    # --- 5. Run Test Loop ---
    while True:
        # Get frames from both cameras
        try:
            frames1 = pipelines['cam1'].wait_for_frames()
            frames2 = pipelines['cam2'].wait_for_frames()
            
            aligned_frames1 = aligners['cam1'].process(frames1)
            aligned_frames2 = aligners['cam2'].process(frames2)
            
            color_frame1 = aligned_frames1.get_color_frame()
            depth_frame1 = aligned_frames1.get_depth_frame()
            color_frame2 = aligned_frames2.get_color_frame()
            depth_frame2 = aligned_frames2.get_depth_frame()

            if not all([color_frame1, depth_frame1, color_frame2, depth_frame2]):
                continue
        except Exception as e:
            logging.warning(f"Failed to get frames: {e}")
            continue

        color_image1 = np.asanyarray(color_frame1.get_data())
        depth_image1 = np.asanyarray(depth_frame1.get_data())
        color_image2 = np.asanyarray(color_frame2.get_data())
        depth_image2 = np.asanyarray(depth_frame2.get_data())

        # --- Extract 3D Features ---
        features1 = extractor1.extract_scene_features(color_image1, depth_image1)
        features2 = extractor2.extract_scene_features(color_image2, depth_image2)
        
        tube_feats1 = features1[0:5]
        peg_feats1 = features1[5:10]
        tube_feats2 = features2[0:5]
        peg_feats2 = features2[5:10]

        # --- Print 3D Coordinates & Cross-Validation ---
        print("\n" + "="*50)
        
        if tube_feats1[4] > 0: # Check cam1 tube flag
            logging.info(f"✅ Cam1 Tube [3D]:   {tube_feats1[0:3]} (Conf: {tube_feats1[3]:.2f})")
        else: logging.info("❌ Cam1 Tube [3D]:   Not detected")
            
        if tube_feats2[4] > 0: # Check cam2 tube flag
            logging.info(f"✅ Cam2 Tube [3D]:   {tube_feats2[0:3]} (Conf: {tube_feats2[3]:.2f})")
        else: logging.info("❌ Cam2 Tube [3D]:   Not detected")

        if peg_feats1[4] > 0: # Check cam1 peg flag
            logging.info(f"✅ Cam1 Peg [3D]:    {peg_feats1[0:3]} (Conf: {peg_feats1[3]:.2f})")
        else: logging.info("❌ Cam1 Peg [3D]:    Not detected")
            
        if peg_feats2[4] > 0: # Check cam2 peg flag
            logging.info(f"✅ Cam2 Peg [3D]:    {peg_feats2[0:3]} (Conf: {peg_feats2[3]:.2f})")
        else: logging.info("❌ Cam2 Peg [3D]:    Not detected")

        # --- 3D Validation Logic ---
        if tube_feats1[4] > 0 and tube_feats2[4] > 0:
            tube_dist = np.linalg.norm(tube_feats1[0:3] - tube_feats2[0:3]) * 100 # in cm
            logging.info(f"   -> 📏 Tube 3D Position Error (Cam1 vs Cam2): {tube_dist:.2f} cm")
        if peg_feats1[4] > 0 and peg_feats2[4] > 0:
            peg_dist = np.linalg.norm(peg_feats1[0:3] - peg_feats2[0:3]) * 100 # in cm
            logging.info(f"   -> 📏 Peg 3D Position Error (Cam1 vs Cam2): {peg_dist:.2f} cm")

        # --- Visualization ---
        results1 = extractor1.model(color_image1, verbose=False, conf=extractor1.confidence_threshold)[0]
        results2 = extractor2.model(color_image2, verbose=False, conf=extractor2.confidence_threshold)[0]
        annotated_img1 = results1.plot()
        annotated_img2 = results2.plot()
        
        combined_img = np.hstack((annotated_img1, annotated_img2))
        cv2.imshow("3D Keypoint Test - Camera 1 (Left) vs Camera 2 (Right)", combined_img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for p in pipelines.values(): p.stop()
    cv2.destroyAllWindows()
    logging.info("Test finished.")

if __name__ == "__main__":
    main()

