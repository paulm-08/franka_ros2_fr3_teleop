# model_pipeline/keypoint_extractor.py
import torch
import numpy as np
import json
from ultralytics import YOLO
import logging

class KeypointExtractor:
    """
    Extracts 2D or 3D keypoints for specified objects using a YOLO detector.
    - In 2D mode, returns (u, v, confidence, detection_flag).
    - In 3D mode, returns (x, y, z, confidence, detection_flag).
    """
    def __init__(self, model_path, confidence_threshold=0.1, use_3d=False, intrinsics_path=None, extrinsics_path=None, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.use_3d = use_3d
        self.confidence_threshold = confidence_threshold

        # --- Model and Class Setup ---
        self.model = YOLO(model_path).to(self.device)
        self.class_names = self.model.names
        self.target_classes = ['tube_tip', 'peg'] 
        
        # --- Dimensioning ---
        # (x,y,z, conf, flag) = 5 features
        # (u,v, conf, flag) = 4 features
        self.feature_dim_per_object = (3 if self.use_3d else 2) + 2
        self.output_dim = len(self.target_classes) * self.feature_dim_per_object

        # --- 3D-Specific Setup ---
        self.K = None
        self.T_base_cam = None
        if self.use_3d:
            logging.info("KeypointExtractor running in ROBUST 3D mode.")
            if intrinsics_path is None or extrinsics_path is None:
                raise ValueError("Intrinsics and extrinsics paths must be provided for 3D mode.")
            
            try:
                with open(intrinsics_path, 'r') as f:
                    intr = json.load(f)
                self.K = np.array([
                    [intr['fx'], 0, intr['ppx']],
                    [0, intr['fy'], intr['ppy']],
                    [0, 0, 1]
                ])
                self.T_base_cam = np.load(extrinsics_path)
            except FileNotFoundError as e:
                logging.error(f"Failed to load calibration file: {e}")
                raise
        else:
            logging.info("KeypointExtractor running in 2D mode.")
        
        logging.info(f"Extractor initialized. Output dim per camera: {self.output_dim}")

    @torch.no_grad()
    def extract_scene_features(self, color_img, depth_img=None):
        """
        Detects objects and returns their keypoints as a single feature vector.
        `depth_img` is only used if `use_3d` is True.
        """
        if color_img is None:
            return np.zeros(self.output_dim, dtype=np.float32)

        results = self.model(color_img, verbose=False, conf=self.confidence_threshold)[0]
        img_h, img_w = color_img.shape[:2]
        
        # --- Find the MOST confident detection for each class ---
        best_detections = {}
        for box in results.boxes:
            class_id = int(box.cls)
            class_name = self.class_names[class_id]
            confidence = float(box.conf)
            
            if class_name in self.target_classes:
                if class_name not in best_detections or confidence > best_detections[class_name]['conf']:
                    best_detections[class_name] = {'box': box, 'conf': confidence}

        # --- Convert best detections to keypoints ---
        detected_kps = {}
        for class_name, detection in best_detections.items():
            xyxy = detection['box'].xyxy[0].cpu().numpy()
            confidence = detection['conf']
            u = (xyxy[0] + xyxy[2]) / 2
            v = (xyxy[1] + xyxy[3]) / 2
                    
            if self.use_3d:
                # --- 3D Logic ---
                # Default to a zero vector (of the correct size)
                detected_kps[class_name] = np.zeros(self.feature_dim_per_object, dtype=np.float32)
                
                if depth_img is None or self.K is None:
                    logging.warning("3D mode enabled but depth_img or intrinsics are None. Returning zeros.")
                    continue # Go to the next detection
                
                # Clamp coordinates to be safely within the image bounds
                u_clamped = int(np.clip(u, 0, img_w - 1))
                v_clamped = int(np.clip(v, 0, img_h - 1))
                
                # Robust depth: average a small 5x5 patch around the center
                y1, y2 = max(0, v_clamped - 2), min(img_h, v_clamped + 3)
                x1, x2 = max(0, u_clamped - 2), min(img_w, u_clamped + 3)
                depth_patch = depth_img[y1:y2, x1:x2]
                valid_depths = depth_patch[depth_patch > 0]
                
                if valid_depths.size == 0:
                    logging.warning(f"No valid depth pixels found for {class_name} at ({u_clamped}, {v_clamped}).")
                    continue # Invalid depth, keep the zero vector

                Z_cam = np.median(valid_depths) / 1000.0 # Use median for robustness
                
                X_cam = (u - self.K[0, 2]) * Z_cam / self.K[0, 0]
                Y_cam = (v - self.K[1, 2]) * Z_cam / self.K[1, 1]
                
                P_cam_h = np.array([X_cam, Y_cam, Z_cam, 1.0])
                P_base = (self.T_base_cam @ P_cam_h)[:3]
                
                # Assign the full 5D vector
                detected_kps[class_name] = np.array([P_base[0], P_base[1], P_base[2], confidence, 1.0])
            else:
                # --- 2D Logic ---
                norm_u = u / img_w
                norm_v = v / img_h
                detected_kps[class_name] = np.array([norm_u, norm_v, confidence, 1.0])

        # --- Construct the final feature vector in a consistent order ---
        feature_vec = []
        for class_name in self.target_classes:
            # This .get() is now robust for both 2D and 3D modes.
            # If a detection failed, it inserts a zero vector of the correct dimension.
            coords_with_metadata = detected_kps.get(class_name, np.zeros(self.feature_dim_per_object))
            feature_vec.extend(coords_with_metadata)

        return np.array(feature_vec, dtype=np.float32)
