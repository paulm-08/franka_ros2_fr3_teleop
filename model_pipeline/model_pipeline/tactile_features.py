"""
tactile_features.py

- Interactive browsing of dataset frames with PCA overlay (centroid, major+minor axis).
- Export tactile features (centroid_x, centroid_y, major_x, major_y) + actions -> .npz
- Works for both dummy and real datasets (detects all *raw_image.jpg tactile images per frame).
- Robust preprocessing, weighted PCA via weighted covariance.
"""
import os
import glob
import cv2
import numpy as np
import logging
import argparse
import yaml
import platform
from pathlib import Path
import json
import torch

from model_pipeline.visual_embedder import VisualEmbedder
from model_pipeline.utils import load_frame_paths, load_actions, get_cfg_path, init_sensor, make_binary_mask, compute_PCA_weighted
from tact9d.shape_reconstruction.sensor import Sensor   # if you rename 9dtact → tact9d

# ---------------- Logger ----------------
logging.basicConfig(
    level=logging.INFO,
    # format="%(asctime)s [%(levelname)s] %(message)s",
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

# ---------------- Single tactile image ----------------
def process_tactile_image(img, ref_img=None, use_height_map=False, sensor=None, logger=False):
    """
    Process a single tactile frame:
      - If use_height_map=True: builds height map using Sensor class
      - Else: uses ref_img difference method
      - Returns visualization and PCA features
    """
    if img is None:
        logging.warning("Input image is None")
        return None, None, None, None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    expected_h, expected_w = 345, 460  # unified output size for display

    height_map = None
    diff_norm = None

    # ---------------- Height map path ----------------
    if use_height_map:
        if (ref_img is None) or (sensor is None):
            logging.warning("No reference image or sensor provided; fallback to raw preprocessing")
            use_height_map = False
        else:
            height_map = sensor.raw_image_2_height_map(gray)
            height_map = sensor.expand_image(height_map)

            # if not is_valid_contact_region(height_map):
            #     logging.warning("Discarded frame: invalid height map contact region.")
            #     vis_height = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            #     vis_height = cv2.applyColorMap(vis_height, cv2.COLORMAP_JET)
            #     vis_height = cv2.resize(vis_height, (expected_w, expected_h))
            #     vis_raw = cv2.resize(img, (expected_w, expected_h))
            #     blank = np.zeros_like(vis_height)
            #     combined = np.hstack([vis_raw, vis_height, blank])
            #     return combined, np.zeros_like(vis_height), None, None

            # Normalize for mask computation
            diff_norm = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX)

    # ---------------- Reference diff path ----------------
    if not use_height_map:
        if ref_img is not None:
            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            diff = ref_gray.astype(np.float32) - gray.astype(np.float32)
            diff[diff < 0] = 0
            diff_norm = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)
        else:
            if logger:
                logging.warning("No reference image provided; using raw grayscale.")
            diff_norm = gray.copy()

    diff_norm = diff_norm.astype(np.uint8)
    gray_inv = cv2.bitwise_not(diff_norm)

    # ---------------- Segmentation ----------------
    if use_height_map:
        # use height map positive region as mask
        if logger:
            logging.info(f"Height map stats: min={height_map.min():.4f}, max={height_map.max():.4f}")
        mask = (height_map > 0.2).astype(np.uint8) * 255  # ✅ ensure uint8 type for OpenCV
    else:
        mask = make_binary_mask(gray_inv)

    coords_rc = np.column_stack(np.where(mask > 0))

    if coords_rc.size < 100:
        if logger:
            logging.warning(f"No contact region detected ({coords_rc.shape[0]} pixels).")
        vis_raw = cv2.resize(img, (expected_w, expected_h))
        vis_mask = cv2.resize(mask, (expected_w, expected_h))
        vis_mask_color = cv2.cvtColor(vis_mask, cv2.COLOR_GRAY2BGR)

        if height_map is not None:
            vis_height = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            vis_height = cv2.applyColorMap(vis_height, cv2.COLORMAP_JET)
            vis_height = cv2.resize(vis_height, (expected_w, expected_h))
            combined = np.hstack([vis_raw, vis_height, vis_mask_color])
        else:
            combined = np.hstack([vis_raw, vis_mask_color])

        return combined, vis_mask_color, None, None

    # ---------------- PCA computation ----------------
    pts_xy = coords_rc[:, [1, 0]].astype(np.float64)
    weights = 255.0 - gray_inv[coords_rc[:, 0], coords_rc[:, 1]].astype(np.float64)

    major, minor, mean_xy, eigvals = compute_PCA_weighted(pts_xy, weights=weights)
    if major is None:
        return img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), None, None

    cx, cy = float(mean_xy[0]), float(mean_xy[1])

    vis_res = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.drawMarker(vis_res, (int(cx), int(cy)), (0, 0, 255),
                   markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)
    scale = int(max(vis_res.shape[:2]) * 0.2)
    cv2.line(vis_res,
             (int(cx - major[0]*scale), int(cy - major[1]*scale)),
             (int(cx + major[0]*scale), int(cy + major[1]*scale)),
             (255, 0, 0), 2)
    cv2.line(vis_res,
             (int(cx - minor[0]*scale), int(cy - minor[1]*scale)),
             (int(cx + minor[0]*scale), int(cy + minor[1]*scale)),
             (0, 255, 0), 2)

    # Resize visuals
    vis_res = cv2.resize(vis_res, (expected_w, expected_h))
    vis_raw = cv2.resize(img, (expected_w, expected_h))
    vis_mask_color = cv2.cvtColor(cv2.resize(mask, (expected_w, expected_h)), cv2.COLOR_GRAY2BGR)

    # Height map visualization
    if height_map is not None:
        vis_height = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        vis_height = cv2.applyColorMap(vis_height, cv2.COLORMAP_JET)
        vis_height = cv2.resize(vis_height, (expected_w, expected_h))
        combined = np.hstack([vis_raw, vis_height, vis_mask_color])
    else:
        combined = np.hstack([vis_raw, vis_mask_color, vis_res])

    return combined, vis_res, np.array([cx, cy]), major
