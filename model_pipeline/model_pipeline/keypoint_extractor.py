# model_pipeline/keypoint_extractor.py
import torch
import numpy as np
import json
from ultralytics import YOLO

class KeypointExtractor:
    """
    Extracts 2D or 3D keypoints for specified objects using a YOLO detector.
    - In 2D mode, it returns normalized pixel coordinates (u, v).
    - In 3D mode, it returns 3D coordinates (x, y, z) in the robot base frame.
    """
    def __init__(self, model_path, confidence_threshold=0.1, use_3d=False, intrinsics_path=None, extrinsics_path=None, device=None):
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.use_3d = use_3d
        
        # --- FIX: Store the confidence threshold ---
        self.confidence_threshold = confidence_threshold

        # --- Model and Class Setup ---
        self.model = YOLO(model_path).to(self.device)
        self.class_names = self.model.names
        # FIX: Ensure target classes match the data.yaml names exactly.
        self.target_classes = ['tube_tip', 'peg'] 
        
        # --- (The rest of the __init__ method is the same) ---
        self.feature_dim_per_object = (3 if self.use_3d else 2) + 2 # (x,y,z) or (u,v) + confidence + detection flag
        self.output_dim = len(self.target_classes) * self.feature_dim_per_object

        # --- 3D-Specific Setup ---
        self.K = None
        self.T_base_cam = None
        if self.use_3d:
            print("KeypointExtractor running in 3D mode.")
            # In 3D mode, camera parameters are required.
            if intrinsics_path is None or extrinsics_path is None:
                raise ValueError("Intrinsics and extrinsics paths must be provided for 3D mode.")
            
            with open(intrinsics_path, 'r') as f:
                intr = json.load(f)
            self.K = np.array([
                [intr['fx'], 0, intr['ppx']],
                [0, intr['fy'], intr['ppy']],
                [0, 0, 1]
            ])
            self.T_base_cam = np.load(extrinsics_path)
        else:
            print("KeypointExtractor running in 2D mode.")

    @torch.no_grad()
    def extract_scene_features(self, color_img, depth_img=None):
        """
        Detects objects and returns their keypoints as a single feature vector.
        `depth_img` is only used if `use_3d` is True.
        """
        if color_img is None:
            return np.zeros(self.output_dim, dtype=np.float32)

        # --- Use the configured confidence threshold during inference ---
        results = self.model(color_img, verbose=False, conf=self.confidence_threshold)[0]
        img_h, img_w = color_img.shape[:2]
        
        # --- Find the MOST confident detection for each class ---
        # This logic is more robust than just taking the first one.
        best_detections = {}
        for box in results.boxes:
            class_id = int(box.cls)
            class_name = self.class_names[class_id]
            confidence = float(box.conf)
            
            if class_name in self.target_classes:
                if class_name not in best_detections or confidence > best_detections[class_name]['conf']:
                    best_detections[class_name] = {'box': box, 'conf': confidence}

        # --- Find the most confident detection for each target class ---
        detected_kps = {}
        for class_name, detection in best_detections.items():
            xyxy = detection['box'].xyxy[0].cpu().numpy()
            confidence = detection['conf']
            u = (xyxy[0] + xyxy[2]) / 2
            v = (xyxy[1] + xyxy[3]) / 2
                    
            if self.use_3d:
                # --- 3D Logic ---
                if depth_img is None or self.K is None:
                    # Cannot proceed without depth/intrinsics in 3D mode
                    detected_kps[class_name] = np.zeros(3)
                    continue
                
                Z_cam = depth_img[int(v), int(u)] / 1000.0 # to meters
                if Z_cam <= 0: # Invalid depth
                    detected_kps[class_name] = np.zeros(3)
                    continue

                X_cam = (u - self.K[0, 2]) * Z_cam / self.K[0, 0]
                Y_cam = (v - self.K[1, 2]) * Z_cam / self.K[1, 1]
                P_cam_h = np.array([X_cam, Y_cam, Z_cam, 1.0])
                P_base = (self.T_base_cam @ P_cam_h)[:3]
                detected_kps[class_name] = P_base
                detected_kps[class_name] = np.concatenate([P_base, [confidence, 1.0]]) # (x,y,z,conf,detected_flag)
            else:
                # --- 2D Logic ---
                # Normalize coordinates to [0, 1] range. Crucial for NNs.
                norm_u = u / img_w
                norm_v = v / img_h
                detected_kps[class_name] = np.array([norm_u, norm_v, confidence, 1.0]) # (u,v,conf,detected_flag)

        # --- Construct the final feature vector in a consistent order ---
        feature_vec = []
        for class_name in self.target_classes:
            coords_with_metadata = detected_kps.get(class_name, np.zeros(self.feature_dim_per_object))
            feature_vec.extend(coords_with_metadata)

        return np.array(feature_vec, dtype=np.float32)