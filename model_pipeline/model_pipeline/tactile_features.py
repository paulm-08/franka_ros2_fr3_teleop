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

from model_pipeline import tactile_features
from tact9d.shape_reconstruction.sensor import Sensor   # if you rename 9dtact → tact9d

# ---------------- Logger ----------------
logging.basicConfig(
    level=logging.INFO,
    # format="%(asctime)s [%(levelname)s] %(message)s",
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

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

def init_sensor(cfg_path, package_share_path=None, calibrated=True, ref=None, open_camera=False):    
    if not os.path.exists(cfg_path):
        raise FileNotFoundError(f"Configuration file not found: {cfg_path}")
    sensor_f = open(cfg_path, 'r+', encoding='utf-8')
    sensor_cfg = yaml.load(sensor_f, Loader=yaml.FullLoader)
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

import cv2
import numpy as np
import logging

def is_valid_contact_region(
    height_map,
    min_rel_area=0.005,
    max_rel_area=0.3,
    base_min_anisotropy=4.0,
    min_height_std=0.001,
    min_region_frac=0.5,
    debug_viz=True
):
    """
    Evaluate whether a height map plausibly represents a tube contact.
    Adapts thresholds based on contact intensity and returns visualization if debug_viz=True.
    """

    if height_map is None:
        logging.warning("No height map provided.")
        return False, None

    h, w = height_map.shape
    total_area = h * w

    # Normalize for consistency
    norm_hmap = height_map - np.min(height_map)
    max_val = np.max(norm_hmap)
    if max_val > 0:
        norm_hmap /= max_val

    # Contact intensity metrics
    contact_intensity = np.mean(norm_hmap[norm_hmap > 0.05]) if np.any(norm_hmap > 0.05) else 0
    contact_std = np.std(norm_hmap)
    contact_strength = contact_intensity + contact_std  # overall signal strength heuristic

    # Adaptive parameters
    # - if contact is strong → relax shape constraints
    # - if contact is weak → demand clear alignment and compactness
    min_anisotropy = base_min_anisotropy + max(0, 2.0 - 10 * contact_strength)
    min_region_frac = 0.5 if contact_strength > 0.1 else 0.7
    min_rel_area_eff = min_rel_area * (0.5 if contact_strength > 0.2 else 1.0)

    # Binary mask
    mask = (norm_hmap > 0.1).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num_labels <= 1:
        return False, cv2.cvtColor((norm_hmap * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR) if debug_viz else (False, None)

    # Sort by area
    areas = stats[1:, cv2.CC_STAT_AREA]
    largest_idx = np.argmax(areas) + 1
    largest_area = areas[largest_idx - 1]
    rel_area = largest_area / total_area
    total_contact_area = np.sum(areas)
    region_frac = largest_area / (total_contact_area + 1e-6)

    # PCA
    ys, xs = np.where(labels == largest_idx)
    pts = np.stack((xs, ys), axis=1).astype(np.float32)
    cov = np.cov(pts.T)
    eigvals, eigvecs = np.linalg.eig(cov)
    eigvals = np.sort(eigvals)[::-1]
    anisotropy = eigvals[0] / (eigvals[1] + 1e-6)
    cx, cy = np.mean(xs), np.mean(ys)
    major_vec = eigvecs[:, np.argmax(eigvals)]

    # Height stats in region
    region_heights = norm_hmap[labels == largest_idx]
    region_std = region_heights.std()
    region_mean = region_heights.mean()

    # Decision logic
    valid = True
    reasons = []

    if not (min_rel_area_eff <= rel_area <= max_rel_area):
        valid = False
        reasons.append(f"Area {rel_area:.3f} out of range.")
    if anisotropy < min_anisotropy:
        valid = False
        reasons.append(f"Anisotropy {anisotropy:.2f} < {min_anisotropy:.2f}")
    if region_frac < min_region_frac:
        valid = False
        reasons.append(f"Fragmented ({region_frac*100:.1f}% main region)")
    if region_std < min_height_std:
        valid = False
        reasons.append(f"Flat region std {region_std:.4f} < {min_height_std}")

    # Visualization
    if debug_viz:
        vis = cv2.cvtColor((norm_hmap * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        vis_mask = cv2.applyColorMap((mask * 255).astype(np.uint8), cv2.COLORMAP_JET)
        vis = cv2.addWeighted(vis, 0.7, vis_mask, 0.3, 0)

        # Draw PCA axis
        scale = int(max(h, w) * 0.2)
        p1 = (int(cx - major_vec[0]*scale), int(cy - major_vec[1]*scale))
        p2 = (int(cx + major_vec[0]*scale), int(cy + major_vec[1]*scale))
        cv2.line(vis, p1, p2, (255, 255, 255), 2)

        # Annotate text
        status = "VALID" if valid else "INVALID"
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.putText(vis, f"{status}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(vis, f"Area={rel_area:.3f}  Anis={anisotropy:.2f}  Frac={region_frac:.2f}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(vis, f"Mean={region_mean:.3f} Std={region_std:.3f} Int={contact_strength:.3f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        if reasons:
            y = 95
            for r in reasons:
                cv2.putText(vis, r, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 150, 255), 1)
                y += 18
    else:
        vis = None

    return valid, vis



# ---------------- Single tactile image ----------------
def process_tactile_image(img, ref_img=None, use_height_map=False, sensor=None):
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

            if not is_valid_contact_region(height_map):
                logging.warning("Discarded frame: invalid height map contact region.")
                vis_height = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                vis_height = cv2.applyColorMap(vis_height, cv2.COLORMAP_JET)
                vis_height = cv2.resize(vis_height, (expected_w, expected_h))
                vis_raw = cv2.resize(img, (expected_w, expected_h))
                blank = np.zeros_like(vis_height)
                combined = np.hstack([vis_raw, vis_height, blank])
                return combined, np.zeros_like(vis_height), None, None

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
            logging.warning("No reference image provided; using raw grayscale.")
            diff_norm = gray.copy()

    diff_norm = diff_norm.astype(np.uint8)
    gray_inv = cv2.bitwise_not(diff_norm)

    # ---------------- Segmentation ----------------
    if use_height_map:
        # use height map positive region as mask
        mask = (height_map > 0).astype(np.uint8) * 255  # ✅ ensure uint8 type for OpenCV
    else:
        mask = make_binary_mask(gray_inv)

    coords_rc = np.column_stack(np.where(mask > 0))

    if coords_rc.size < 100:
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

# ---------------- Export NPZ ----------------
def build_npz_dataset(data_dir="dataset", out_file="data/processed/dataset_features.npz", use_height_map=False):
    frame_dirs = load_frame_paths(data_dir)
    all_features, actions = [], load_actions(frame_dirs)

    # Load reference tactile images from frame_0
    ref_dir = frame_dirs[0]
    ref_tactile = {os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
                   for p in glob.glob(os.path.join(ref_dir, "*raw_image.jpg"))}

    for frame_dir in frame_dirs:
        tactile_imgs = glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))
        frame_feats = []

        for img_path in tactile_imgs:
            fname = os.path.basename(img_path)
            img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
            ref = ref_tactile.get(fname, None)

            _, _, centroid, major = process_tactile_image(img, ref_img=ref, use_height_map=use_height_map)
            if centroid is None or major is None:
                frame_feats.extend([0.0, 0.0, 0.0, 0.0])
            else:
                frame_feats.extend([centroid[0], centroid[1], major[0], major[1]])

        all_features.append(frame_feats)

    X = np.array(all_features, dtype=np.float32)
    y = np.array(actions, dtype=np.float32)
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    np.savez(out_file, tactile=X, actions=y)
    logging.info(f"✅ Saved dataset with {len(frame_dirs)} samples → {out_file}")

# ---------------- Browsing ----------------
def interactive_browse(data_dir="C:\\Users\\paulm\\franka_ros2_ws\\src\\model_pipeline\\dataset_real_full", use_height_map=False):
    frame_dirs = load_frame_paths(data_dir)
    if not frame_dirs:
        logging.error(f"No frames found in {data_dir}")
        return
    
    # Reference tactile images
    ref_dir = frame_dirs[0]
    ref_tactile = {os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
                   for p in glob.glob(os.path.join(ref_dir, "*raw_image.jpg"))}

    if use_height_map and ref_tactile:
        logging.info("Using reference tactile images for preprocessing")
        index_sensor = init_sensor(
            cfg_path="C:\\Users\\paulm\\franka_ros2_ws\\src\\tact9d\\tact9d\\shape_reconstruction\\shape_config_index.yaml",
            calibrated=True,
            ref=ref_tactile.get("rindex_raw_image.jpg"),
            open_camera=False)
        middle_sensor = init_sensor(
            cfg_path="C:\\Users\\paulm\\franka_ros2_ws\\src\\tact9d\\tact9d\\shape_reconstruction\\shape_config_middle.yaml",
            calibrated=True,
            ref=ref_tactile.get("rmiddle_raw_image.jpg"),
            open_camera=False)
        thumb_sensor = init_sensor(
            cfg_path="C:\\Users\\paulm\\franka_ros2_ws\\src\\tact9d\\tact9d\\shape_reconstruction\\shape_config_thumb.yaml",
            calibrated=True,
            ref=ref_tactile.get("rthumb_raw_image.jpg"),
            open_camera=False)
    elif use_height_map:
        logging.warning("No reference tactile images found; cannot use height map preprocessing")
        use_height_map = False

    idx = 0
    while True:
        frame_dir = frame_dirs[idx]
        tactile_imgs = glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))
        vis_list = []

        logging.info(f"Frame {idx+1}/{len(frame_dirs)} | 'n': next, 'p': prev, 'q': quit")

        for img_path in tactile_imgs:
            logging.info(f" Processing {img_path.replace(frame_dir, '')}")
            fname = os.path.basename(img_path)
            img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
            ref = ref_tactile.get(fname, None)
            if use_height_map:
                if "rindex" in fname.lower():
                    sensor = index_sensor
                elif "rmiddle" in fname.lower():
                    sensor = middle_sensor
                elif "rthumb" in fname.lower():
                    sensor = thumb_sensor
                else:
                    sensor = None
            else:
                sensor = None
            vis_raw, vis_res, _, _ = process_tactile_image(img, ref_img=ref, use_height_map=use_height_map, sensor=sensor)
            
            if vis_raw is not None:
                combined = np.hstack([vis_raw, vis_res])
                vis_list.append(combined)

        if vis_list:
            canvas = np.vstack(vis_list)

            # Resize to fit screen (~1000 px max width)
            max_width = 1000
            scale = max_width / canvas.shape[1]
            if scale < 1.0:
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale,
                                    interpolation=cv2.INTER_AREA)

            cv2.imshow("Tactile PCA (browse)", canvas)
        else:
            logging.warning(f"No tactile images found in {frame_dir}")

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("n"):
            idx = (idx + 1) % len(frame_dirs)
        elif key == ord("p"):
            idx = (idx - 1) % len(frame_dirs)

    cv2.destroyAllWindows()
# ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["browse", "export"], default="browse")
    parser.add_argument("--data_dir", default="C:\\Users\\paulm\\franka_ros2_ws\\src\\model_pipeline\\dataset_real_full")
    parser.add_argument("--out_file", default="data/processed/dataset_features.npz")
    parser.add_argument("--height_map", action="store_true",
                        help="Use 9DTact height map instead of raw image")
    args = parser.parse_args()

    if args.mode == "browse":
        interactive_browse(args.data_dir, use_height_map=args.height_map)
    elif args.mode == "export":
        build_npz_dataset(args.data_dir, args.out_file, use_height_map=args.height_map)

if __name__ == "__main__":
    main()