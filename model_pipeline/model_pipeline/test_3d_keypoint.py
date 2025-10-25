import cv2
import torch
import numpy as np
import logging
from pathlib import Path
import pyrealsense2 as rs
import yaml
import inquirer
import os
import time
import glob
import pickle

# Import your pipeline components
from model_pipeline import paths
from model_pipeline.keypoint_extractor import KeypointExtractor
from model_pipeline.utils import load_frame_paths, load_actions
from model_pipeline.kinematics import get_urdf_string_from_xacro, KinematicsSolver

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
np.set_printoptions(precision=4, suppress=True)

def find_demo_dirs(root_search_path):
    """Recursively finds all valid demonstration directories."""
    logging.info(f"Searching for demonstration directories in: {root_search_path}...")
    found_demos = []
    for dirpath, _, _ in os.walk(root_search_path):
        if glob.glob(os.path.join(dirpath, 'frame_*')):
            relative_path = Path(dirpath).relative_to(paths.WORKSPACE_ROOT)
            found_demos.append(str(relative_path))
    logging.info(f"Found {len(found_demos)} potential demonstration directories.")
    return sorted(found_demos)

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
    state_config = config.get("state", {})
    if not state_config.get("use_keypoint_extractor", False):
        logging.error("This script requires 'use_keypoint_extractor: true' in config.yaml.")
        return

    # --- 2. Interactive Mode Selection ---
    try:
        source_question = [
            inquirer.List('source', message="Select data source", choices=['Live Feed', 'Recorded Dataset']),
        ]
        source_answer = inquirer.prompt(source_question)
        if not source_answer: return
        data_source = source_answer['source']
        
        frame_dirs = []
        if data_source == 'Recorded Dataset':
            demo_choices = find_demo_dirs(paths.RAW_DATA_DIR)
            if not demo_choices: logging.error(f"No demonstration directories found in {paths.RAW_DATA_DIR}."); return
            
            dataset_question = [
                inquirer.List('demo_dir', message="Select the demonstration directory to test on", choices=demo_choices)
            ]
            dataset_answer = inquirer.prompt(dataset_question)
            demo_path = paths.WORKSPACE_ROOT / dataset_answer['demo_dir']
            frame_dirs = load_frame_paths(str(demo_path))
            logging.info(f"Loaded {len(frame_dirs)} frames from {demo_path.name}.")

    except (KeyboardInterrupt, TypeError):
        logging.info("\nSelection cancelled."); return
        
    # --- 3. Initialize Hardware (if Live) or set to None ---
    pipelines = {}
    if data_source == 'Live Feed':
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

    # --- 5. Initialize "Carry Forward" State ---
    coord_dim = 3 if vision_config.get("use_3d", False) else 2
    last_known_coords = {
        'cam1_tube': np.zeros(coord_dim, dtype=np.float32), 'cam1_peg':  np.zeros(coord_dim, dtype=np.float32),
        'cam2_tube': np.zeros(coord_dim, dtype=np.float32), 'cam2_peg':  np.zeros(coord_dim, dtype=np.float32)
    }

    # --- 6. Run Test Loop ---
    idx = 0
    while True:
        if data_source == 'Live Feed':
            # ... (Get live frames from pipelines) ...
            pass
        else: # Recorded Dataset
            if idx >= len(frame_dirs):
                logging.info("End of trajectory reached."); break
            frame_dir = frame_dirs[idx]
            color_image1 = cv2.imread(str(Path(frame_dir) / "color_image1.jpg"))
            depth_image1 = cv2.imread(str(Path(frame_dir) / "depth_image1.png"), cv2.IMREAD_UNCHANGED)
            color_image2 = cv2.imread(str(Path(frame_dir) / "color_image2.jpg"))
            depth_image2 = cv2.imread(str(Path(frame_dir) / "depth_image2.png"), cv2.IMREAD_UNCHANGED)

        # --- Extract Raw 3D Features (same as builder) ---
        raw_feats1 = extractor1.extract_scene_features(color_image1, depth_image1)
        raw_feats2 = extractor2.extract_scene_features(color_image2, depth_image2)
        
        # --- Apply "Carry Forward" Logic (same as builder) ---
        def update_features(feats, cam_id, key_prefix):
            if feats is None: feats = np.zeros(extractor1.output_dim) # Safety
            tube_key, peg_key = f"{key_prefix}_tube", f"{key_prefix}_peg"
            tube_feats, peg_feats = feats[0:5], feats[5:10] # 5D: [x,y,z,conf,flag]

            if tube_feats[4] == 0: tube_feats[0:3] = last_known_coords[tube_key]; tube_feats[3] = 0
            else: last_known_coords[tube_key] = tube_feats[0:3]
            
            if peg_feats[4] == 0: peg_feats[0:3] = last_known_coords[peg_key]; peg_feats[3] = 0
            else: last_known_coords[peg_key] = peg_feats[0:3]
            return tube_feats, peg_feats

        clean_tube1, clean_peg1 = update_features(raw_feats1, 'cam1', 'cam1')
        clean_tube2, clean_peg2 = update_features(raw_feats2, 'cam2', 'cam2')

        # --- Print 3D Coordinates & Cross-Validation ---
        print("\n" + "="*50); logging.info(f"Frame {idx}")
        logging.info(f"  Cam1 Tube: {'✅' if clean_tube1[4]>0 else '❌'} {clean_tube1[0:3]} (Conf: {clean_tube1[3]:.2f})")
        logging.info(f"  Cam2 Tube: {'✅' if clean_tube2[4]>0 else '❌'} {clean_tube2[0:3]} (Conf: {clean_tube2[3]:.2f})")
        logging.info(f"  Cam1 Peg:  {'✅' if clean_peg1[4]>0 else '❌'} {clean_peg1[0:3]} (Conf: {clean_peg1[3]:.2f})")
        logging.info(f"  Cam2 Peg:  {'✅' if clean_peg2[4]>0 else '❌'} {clean_peg2[0:3]} (Conf: {clean_peg2[3]:.2f})")
        
        # --- 3D Validation Logic ---
        if clean_tube1[4] > 0 and clean_tube2[4] > 0:
            tube_dist = np.linalg.norm(clean_tube1[0:3] - clean_tube2[0:3]) * 100
            logging.info(f"   -> 📏 Tube 3D Error: {tube_dist:.2f} cm")
        if clean_peg1[4] > 0 and clean_peg2[4] > 0:
            peg_dist = np.linalg.norm(clean_peg1[0:3] - clean_peg2[0:3]) * 100
            logging.info(f"   -> 📏 Peg 3D Error:  {peg_dist:.2f} cm")

        # --- Visualization ---
        raw_results1 = extractor1.model(color_image1, verbose=False, conf=extractor1.confidence_threshold)[0]
        raw_results2 = extractor2.model(color_image2, verbose=False, conf=extractor2.confidence_threshold)[0]
        annotated_img1 = raw_results1.plot()
        annotated_img2 = raw_results2.plot()
        
        combined_img = np.hstack((annotated_img1, annotated_img2))
        cv2.imshow("3D Keypoint Test - Cam 1 (Raw) vs Cam 2 (Raw)", combined_img)
        
        key = cv2.waitKey(0 if data_source == 'Recorded Dataset' else 1) & 0xFF
        if key == ord('q'): break
        elif key == ord('n'): idx = (idx + 1)
        elif key == ord('p') and data_source == 'Recorded Dataset': idx = max(0, idx - 1)

    if data_source == 'Live Feed':
        for p in pipelines.values(): p.stop()
    cv2.destroyAllWindows()
    logging.info("Test finished.")

if __name__ == "__main__":
    main()

