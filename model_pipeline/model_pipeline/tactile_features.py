"""
tactile_features.py (Revised)

- Extracts a rich 8D feature vector from tactile images.
- Handles "no contact" states unambiguously with a contact flag.
- Separates feature extraction from visualization for performance.
- Uses robust, adaptive segmentation for both processing paths.
"""
import cv2
import numpy as np
import logging

# Assuming utils.py is in the same directory or Python path
from model_pipeline.utils import make_binary_mask, compute_PCA_weighted
from model_pipeline import paths
from model_pipeline.utils import find_demo_dirs

# It's good practice to define the feature dimension as a constant
TACTILE_FEATURE_DIM = 8

def process_tactile_image(img, ref_img=None, use_height_map=False, sensor=None, min_contact_pixels=100, generate_visualization=False):
    """
    Processes a single tactile image to extract a rich feature vector.

    Args:
        img (np.ndarray): The input BGR image from the tactile sensor.
        ref_img (np.ndarray, optional): The reference (no contact) image.
        use_height_map (bool): If True, uses the sensor object to compute a height map.
        sensor (Sensor, optional): The sensor object needed for height map computation.
        min_contact_pixels (int): The minimum number of pixels in a mask to be considered contact.
        generate_visualization (bool): If True, creates and returns visualization images.

    Returns:
        tuple: A tuple containing:
            - np.ndarray: The 8D feature vector. Zeros if no contact.
            - np.ndarray: The visualization image, or None if generate_visualization is False.
    """
    if img is None:
        return np.zeros(TACTILE_FEATURE_DIM, dtype=np.float32), None

    # --- 1. Generate Normalized Grayscale Pressure Map ---
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    height_map = None
    diff_norm = None

    if use_height_map:
        if sensor is None:
            logging.warning("Sensor object not provided; falling back to reference difference method.")
            use_height_map = False
        else:
            height_map = sensor.raw_image_2_height_map(gray)
            # Normalize height map to [0, 255] to create a pressure map
            diff_norm = cv2.normalize(height_map, None, 0, 255, cv2.NORM_MINMAX)
    
    if not use_height_map:
        if ref_img is not None:
            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            # Difference shows indentation
            diff = ref_gray.astype(np.float32) - gray.astype(np.float32)
            diff[diff < 0] = 0 # Ignore pixels that got brighter
            diff_norm = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)
        else:
            # Fallback to using raw grayscale if no reference is available
            diff_norm = gray.copy()
    
    diff_norm = diff_norm.astype(np.uint8)
    
    # --- 2. Robust Segmentation ---
    # Invert the image because contact is typically darker (higher pressure value)
    # The segmentation function expects the object of interest to be white.
    gray_inv = cv2.bitwise_not(diff_norm)
    # ---------------- Segmentation ----------------
    if use_height_map:
        # use height map positive region as mask
        # logging.info(f"Height map stats: min={height_map.min():.4f}, max={height_map.max():.4f}")
        mask = (height_map > 0.2).astype(np.uint8) * 255  # ✅ ensure uint8 type for OpenCV
    else:
        mask = make_binary_mask(gray_inv)    
    # --- 3. Check for Contact ---
    coords_rc = np.column_stack(np.where(mask > 0))
    if coords_rc.shape[0] < min_contact_pixels:
        # No contact or insignificant contact
        return np.zeros(TACTILE_FEATURE_DIM, dtype=np.float32), None

    # --- 4. Compute Weighted PCA for Features ---
    pts_xy = coords_rc[:, [1, 0]].astype(np.float64) # Convert (row, col) to (x, y)
    # Weights are the pressure values from the normalized difference image
    weights = diff_norm[coords_rc[:, 0], coords_rc[:, 1]].astype(np.float64)

    try:
        major, minor, mean_xy, eigvals = compute_PCA_weighted(pts_xy, weights=weights)
    except (ValueError, np.linalg.LinAlgError):
        # PCA can fail if all points are collinear or other numerical issues
        return np.zeros(TACTILE_FEATURE_DIM, dtype=np.float32), None
    
    # --- 5. Assemble the Feature Vector ---
    feature_vector = np.array([
        mean_xy[0],          # Centroid x
        mean_xy[1],          # Centroid y
        major[0],            # Major axis vector x
        major[1],            # Major axis vector y
        np.log(eigvals[0] + 1),          # Major eigenvalue (variance along major axis -> "length")
        np.log(eigvals[1] + 1),          # Minor eigenvalue (variance along minor axis -> "width")
        height_map.max() if use_height_map else gray_inv.max(),       # Total force proxy
        1.0                  # Contact flag
    ], dtype=np.float32)

    # --- 6. (Optional) Generate Visualization ---
    vis_image = None
    if generate_visualization:
        expected_h, expected_w = 345, 460 # Unified display size
        vis_raw = cv2.resize(img, (expected_w, expected_h))
        vis_mask = cv2.cvtColor(cv2.resize(mask, (expected_w, expected_h)), cv2.COLOR_GRAY2BGR)

        # Draw PCA results on the mask
        cx, cy = int(mean_xy[0]), int(mean_xy[1])
        
        # Scale eigenvectors for visualization
        scale_factor = np.sqrt(eigvals[0]) * 0.5 
        p1 = (int(cx - major[0] * scale_factor), int(cy - major[1] * scale_factor))
        p2 = (int(cx + major[0] * scale_factor), int(cy + major[1] * scale_factor))
        cv2.line(vis_mask, p1, p2, (255, 0, 0), 2) # Major axis in blue

        scale_factor = np.sqrt(eigvals[1]) * 0.5
        p1 = (int(cx - minor[0] * scale_factor), int(cy - minor[1] * scale_factor))
        p2 = (int(cx + minor[0] * scale_factor), int(cy + minor[1] * scale_factor))
        cv2.line(vis_mask, p1, p2, (0, 255, 0), 2) # Minor axis in green

        cv2.drawMarker(vis_mask, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=12, thickness=2)

        # Stack images for a combined view
        vis_image = np.hstack([vis_raw, vis_mask])

    return feature_vector, vis_image


import argparse
import glob
import json
import os
from pathlib import Path

# (Assumes the rest of the imports like cv2, logging, numpy, and the utils are already at the top of the file)
# (Assumes the revised process_tactile_image function is defined above this)


def interactive_browse(data_dir, use_height_map=False):
    """
    Interactively loads and displays tactile data from a directory,
    showing the results of the feature extraction.
    """
    from model_pipeline.utils import load_frame_paths, get_cfg_path, init_sensor # Local import

    # --- Handle clip marks to browse only relevant frames ---
    clip_marks_path = Path(data_dir) / "clip_marks.json"
    if clip_marks_path.exists():
        with open(clip_marks_path, "r") as f:
            clip_marks = json.load(f)
        logging.info("Using clip_marks.json → filtering frames and using per-clip reference images.")
        
        clips = clip_marks if isinstance(clip_marks, list) else clip_marks.values()
        all_frame_dirs = load_frame_paths(data_dir)
        frame_names = [os.path.basename(f) for f in all_frame_dirs]
        
        frame_dirs, ref_frames = [], []
        for clip in clips:
            start, end = clip.get("start"), clip.get("end")
            try:
                i_start, i_end = frame_names.index(start), frame_names.index(end)
                frame_dirs.extend(all_frame_dirs[i_start : i_end + 1])
                ref_frames.append(all_frame_dirs[i_start])
            except (ValueError, KeyError):
                logging.warning(f"Clip range ({start} -> {end}) invalid or not found, skipping.")
    else:
        frame_dirs = load_frame_paths(data_dir)
        ref_frames = [frame_dirs[0]] if frame_dirs else []
        logging.info("No clip_marks.json found → using all frames and a single global reference.")

    if not frame_dirs:
        logging.error(f"No frames to display in {data_dir}.")
        return

    # --- Initialize sensors with the first reference frame ---
    ref_dir = ref_frames[0]
    ref_tactile = {
        os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
        for p in glob.glob(os.path.join(ref_dir, "*raw_image.jpg"))
    }
    
    sensors = {}
    if use_height_map:
        for name in ["index", "middle", "thumb"]:
            ref_img = ref_tactile.get(f"r{name}_raw_image.jpg")
            if ref_img is not None:
                sensors[f"r{name}"] = init_sensor(cfg_path=get_cfg_path(name), calibrated=True, ref=ref_img)

    idx = 0
    current_ref_dir = ref_dir
    
    # --- Main browsing loop ---
    while True:
        frame_dir = frame_dirs[idx]
        
        if frame_dir in ref_frames and frame_dir != current_ref_dir:
            logging.info(f"Updating reference image to frame {os.path.basename(frame_dir)}")
            ref_tactile = {os.path.basename(p): cv2.imread(p) for p in glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))}
            if use_height_map:
                for name, sensor_obj in sensors.items():
                    sensor_obj.update_ref(ref_tactile.get(f"{name}_raw_image.jpg"))
            current_ref_dir = frame_dir

        # --- Print status header (once per frame) ---
        print("\n" + "="*60)
        print(f"Frame {idx+1}/{len(frame_dirs)} ({os.path.basename(frame_dir)}) | 'n': next, 'p': prev, 'q': quit")
        print("="*60)

        vis_list = []
        tactile_img_paths = sorted(glob.glob(os.path.join(frame_dir, "*raw_image.jpg")))

        for img_path in tactile_img_paths:
            fname = os.path.basename(img_path)
            img = cv2.imread(img_path)
            ref = ref_tactile.get(fname)
            
            sensor_key = next((key for key in sensors if key in fname), None)
            sensor = sensors.get(sensor_key) if use_height_map and sensor_key else None
            
            feature_vec, vis_image = process_tactile_image(
                img, ref_img=ref, use_height_map=use_height_map, sensor=sensor, generate_visualization=True
            )

            is_contact = "CONTACT" if feature_vec[-1] > 0 else "NO CONTACT"
            print(f"  > {fname}: {is_contact}")
            print(f"    Features: {np.round(feature_vec, 2)}")
            
            # --- Graceful visualization for NO CONTACT case ---
            if vis_image is None:
                # Create a fallback view: raw image + black panel
                expected_h, expected_w = 345, 460
                vis_raw = cv2.resize(img, (expected_w, expected_h))
                blank_panel = np.zeros_like(vis_raw)
                vis_image = np.hstack([vis_raw, blank_panel])
            
            vis_list.append(vis_image)

        if vis_list:
            canvas = np.vstack(vis_list)

            # --- Add these lines to resize the canvas ---
            max_width = 500  # Adjust this value to your preference (e.g., 1000, 800, 600)
            
            # Only resize if the canvas is wider than the max_width
            if canvas.shape[1] > max_width:
                scale = max_width / canvas.shape[1]
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            # --- End of added lines ---

            cv2.imshow("Tactile Feature Browser", canvas)
        else:
            logging.warning(f"No tactile images found or processed in {frame_dir}")

        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            idx = (idx + 1) % len(frame_dirs)
        elif key == ord('p'):
            idx = (idx - 1 + len(frame_dirs)) % len(frame_dirs) # Correct modulo for negative numbers

    cv2.destroyAllWindows()


def main():
    """ Main entry point to run the interactive browser. """
    parser = argparse.ArgumentParser(description="Utilities for tactile feature processing.")
    parser.add_argument("--mode", choices=["browse"], default="browse", help="Operation to perform.")
    # Use the dynamic path from paths.py as the default
    parser.add_argument("--data_dirs", nargs="+", default=[str(paths.RAW_DATA_DIR)+'/clipped_data/'], help="List of dataset directories.")
    parser.add_argument("--height_map", action="store_true", help="Use 9DTact height map.")
    args = parser.parse_args()

    if args.mode == "browse":
        # We still resolve the path to handle user-provided relative paths
        data_dir = Path(args.data_dirs[0]).expanduser().resolve()
        if not data_dir.exists():
            logging.error(f"Data directory not found: {data_dir}")
            return
        # The interactive_browse function remains the same
        interactive_browse(str(data_dir), use_height_map=args.height_map)

if __name__ == "__main__":
    # Example Usage from your terminal:
    # python tactile_features.py --data_dirs /path/to/your/dataset
    main()