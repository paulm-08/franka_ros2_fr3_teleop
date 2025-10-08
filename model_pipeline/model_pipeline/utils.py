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
from tact9d.shape_reconstruction.sensor import Sensor   # if you rename 9dtact → tact9d

# ---------------- Logger ----------------
logging.basicConfig(
    level=logging.INFO,
    # format="%(asctime)s [%(levelname)s] %(message)s",
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

# ---------------- Path setup ----------------
# Detect the platform
IS_WINDOWS = platform.system().lower().startswith("win")

# Hardcode the repo root based on the OS
if IS_WINDOWS:
    REPO_ROOT = Path("C:/Users/paulm/franka_ros2_ws/src")
else:
    REPO_ROOT = Path("/home/user/franka_ros2_ws/src")

# Define useful paths
DEFAULT_DATA_DIR = REPO_ROOT / "model_pipeline" / "dataset_real_full"
TACT9D_CFG_DIR = REPO_ROOT / "tact9d" / "tact9d" / "shape_reconstruction"

def get_cfg_path(sensor_name):
    """Return full path to the sensor YAML config file."""
    return TACT9D_CFG_DIR / f"shape_config_{sensor_name}.yaml"

# ---------------- Helpers ----------------
def load_frame_paths(data_dir="dataset"):
    frame_dirs = glob.glob(os.path.join(data_dir, "frame_*"))
    return sorted(frame_dirs, key=lambda x: int(os.path.basename(x).split("_")[1]))

def load_actions(frame_dirs):
    actions = []
    for f in frame_dirs:
        joint_file = os.path.join(f, "right_arm_joint.txt")
        if os.path.exists(joint_file):
            with open(joint_file) as fp:
                joints = np.fromstring(fp.read().strip(), sep=" ")
            actions.append(joints)
        else:
            actions.append(np.zeros(7))  # fallback dummy
    return np.array(actions)

# ---------------- Sensor init ----------------
def init_sensor(cfg_path, package_share_path=None, calibrated=True, ref=None, open_camera=False):    
    cfg_path = Path(cfg_path)
    if not cfg_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {cfg_path}")
    with open(cfg_path, 'r', encoding='utf-8') as f:
        sensor_cfg = yaml.safe_load(f)
    tactile_sensor = Sensor(sensor_cfg, package_share_path=package_share_path,
                    calibrated=calibrated, ref=ref, open_camera=open_camera)
    return tactile_sensor

# ---------------- Weighted PCA (numpy) ----------------
def compute_PCA_weighted(pts_xy, weights=None):
    """
    pts_xy: (N,2) array in (x,y) coordinates.
    weights: (N,) non-negative weights (e.g. depth/intensity). If None => uniform weights.
    Returns: major_vec (2,), minor_vec (2,), mean_xy (2,), eigvals (2,)
    """
    pts_xy = np.asarray(pts_xy, dtype=np.float64)
    n = pts_xy.shape[0]
    if n < 2:
        raise ValueError("Need at least 2 points")

    if weights is None:
        weights = np.ones(n, dtype=np.float64)
    else:
        weights = np.asarray(weights, dtype=np.float64)
        # replace negative/NaN with zeros
        weights[~np.isfinite(weights)] = 0.0
        weights[weights < 0] = 0.0

    wsum = weights.sum()
    if wsum <= 0:
        weights = np.ones(n, dtype=np.float64)
        wsum = weights.sum()
    # normalized weights
    weights = weights / wsum

    # weighted mean
    mean = np.average(pts_xy, axis=0, weights=weights)  # shape (2,)
    centered = pts_xy - mean  # (N,2)

    # Weighted covariance: use numpy.cov on (2 x N) with aweights
    try:
        cov = np.cov(centered.T, aweights=weights)  # (2,2)
    except Exception:
        # fallback to manual computation: cov = (centered * weights[:,None]).T @ centered
        cov = (centered * weights[:, None]).T.dot(centered)

    # Numerical safety
    if not np.all(np.isfinite(cov)):
        raise ValueError("Covariance contains non-finite values")

    # Eigen-decomposition
    eigvals, eigvecs = np.linalg.eigh(cov)  # eigvals ascending
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]  # columns are eigenvectors

    major = eigvecs[:, 0]  # (x,y)
    minor = eigvecs[:, 1]

    return major, minor, mean, eigvals

def preprocess_tactile(img, ref_img, threshold=5):
    """
    Compute height map-like difference from ref image.
    """
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)

    diff = ref_gray.astype(np.float32) - img_gray.astype(np.float32)
    diff[diff < threshold] = 0  # remove lighting noise

    # Normalize to [0,255]
    diff_norm = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)
    diff_uint8 = diff_norm.astype(np.uint8)

    return diff_uint8

# ---------------- Image segmentation (robust) ----------------
def make_binary_mask(img_gray):
    """
    Build a binary mask from grayscale tactile image.
    Strategy:
      1. Gaussian blur to reduce noise.
      2. Try Otsu threshold (global): usually good when object darker.
      3. If result too sparse/noisy: fallback to adaptive thresholding.
      4. Morphological opening/closing to remove speckles and fill holes.
    Returns a uint8 binary mask (0/255).
    """
    blur = cv2.GaussianBlur(img_gray, (5, 5), 0)

    # Try Otsu first (invert because shapes are darker -> make shape=255)
    _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # If too few pixels or mask dominated by edges, try adaptive with stronger smoothing
    if np.count_nonzero(mask) < 10:
        # adaptive mean threshold with larger block size
        mask = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 31, 8
        )

    # Morphological cleanup: remove small noise, fill holes
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Remove very small connected components
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels > 1:
        min_size = 20  # pixels
        new_mask = np.zeros_like(mask)
        for lab in range(1, num_labels):
            area = stats[lab, cv2.CC_STAT_AREA]
            if area >= min_size:
                new_mask[labels == lab] = 255
        mask = new_mask

    return mask.astype(np.uint8)

# ---------------- Contact region validation ----------------
def is_valid_contact_region(
    height_map,
    min_rel_area=0.01,
    max_rel_area=0.3,
    min_anisotropy=2.0,
    min_region_frac=0.2,
    min_patch_distance=5,
    min_height_diff=0.002,
    debug_viz=True
):
    """
    Evaluate whether a height map plausibly represents a tube contact.

    Logic:
    - Identify patches above an adaptive threshold.
    - Merge close & aligned patches.
    - Consider a contact valid if:
        * main region is compact and stands out from noise
        * orientation/anistropy is sufficient
        * region fraction is high enough
    - Avoid normalizing height_map to preserve amplitude differences.
    """

    if height_map is None:
        return False, None

    h, w = height_map.shape
    total_area = h * w

    # Compute basic stats
    min_h, max_h = np.min(height_map), np.max(height_map)
    if max_h - min_h < min_height_diff:
        return False, None  # too flat → no contact

    # Adaptive threshold: ignore minor values for strong contacts
    thresh = min_height_diff + 0.2*(max_h - min_h)
    mask = (height_map >= thresh).astype(np.uint8)

    # Morphology to remove tiny noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Connected components
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return False, mask if debug_viz else None

    # Compute distances between patches and merge close ones
    # Here we consider only largest components
    areas = stats[1:, cv2.CC_STAT_AREA]
    sorted_idx = np.argsort(areas)[::-1]
    largest_idx = sorted_idx[0] + 1
    ys, xs = np.where(labels == largest_idx)
    main_pts = np.stack([xs, ys], axis=1).astype(np.float32)

    # Optionally: merge nearby smaller patches
    for idx in sorted_idx[1:]:
        patch_idx = idx + 1
        ys2, xs2 = np.where(labels == patch_idx)
        patch_pts = np.stack([xs2, ys2], axis=1)
        # distance between centroids
        d = np.linalg.norm(np.mean(main_pts, axis=0) - np.mean(patch_pts, axis=0))
        if d <= min_patch_distance:
            main_pts = np.vstack([main_pts, patch_pts])

    # PCA on merged region
    cov = np.cov(main_pts.T)
    eigvals, eigvecs = np.linalg.eig(cov)
    eigvals = np.sort(eigvals)[::-1]
    anisotropy = eigvals[0] / (eigvals[1] + 1e-6)

    # Area & fraction
    main_area = main_pts.shape[0]
    rel_area = main_area / total_area
    total_contact_area = np.sum(areas)
    region_frac_actual = main_area / (total_contact_area + 1e-6)

    # Height stats
    region_heights = height_map[labels == largest_idx]
    region_mean = np.mean(region_heights)
    region_std = np.std(region_heights)

    # Decision logic
    valid = True
    reasons = []

    if not (min_rel_area <= rel_area <= max_rel_area):
        valid = False
        reasons.append(f"Area {rel_area:.3f} out of range")
    if anisotropy < min_anisotropy:
        valid = False
        reasons.append(f"Anisotropy {anisotropy:.2f} too low")
    if region_frac_actual < min_region_frac:
        valid = False
        reasons.append(f"Fragmented region {region_frac_actual:.2f}")
    if region_std < min_height_diff:
        valid = False
        reasons.append(f"Flat region std {region_std:.4f}")

    # Visualization
    vis = None
    if debug_viz:
        vis = cv2.cvtColor((mask*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        # Draw PCA major axis
        cx, cy = np.mean(main_pts, axis=0)
        major_vec = eigvecs[:, np.argmax(eigvals)]
        scale = int(max(h, w) * 0.2)
        p1 = (int(cx - major_vec[0]*scale), int(cy - major_vec[1]*scale))
        p2 = (int(cx + major_vec[0]*scale), int(cy + major_vec[1]*scale))
        cv2.line(vis, p1, p2, (0, 255, 0) if valid else (0, 0, 255), 2)
        status = "VALID" if valid else "INVALID"
        cv2.putText(vis, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if valid else (0,0,255), 2)
        y0 = 50
        for r in reasons:
            cv2.putText(vis, r, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            y0 += 18

    return valid, vis
