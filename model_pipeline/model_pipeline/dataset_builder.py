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
from model_pipeline.tactile_features import process_tactile_image
from model_pipeline.utils import load_frame_paths, load_actions, get_cfg_path, init_sensor
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

def build_npz_dataset(
    data_dir="dataset",
    out_file="data/processed/dataset_features.npz",
    use_height_map=False
):
    """
    Build dataset for rollout-style model training:
        tactile_t + visual_t + joints_t → joints_t+1
    Saves .npz with arrays:
        tactile (N, T_dim)
        visual (N, V_dim)
        actions (N, 7)
    """

    # === Setup ===
    device = "cuda" if torch.cuda.is_available() else "cpu"
    embedder = VisualEmbedder(backbone="resnet18", device=device, pretrained=True, out_dim=256)
    logging.info(f"Visual embedder initialized on {device} with out_dim={embedder.out_dim}")

    # === Clip management ===
    clip_marks_path = os.path.join(data_dir, "clip_marks.json")
    if os.path.exists(clip_marks_path):
        with open(clip_marks_path, "r") as f:
            clip_marks = json.load(f)
        logging.info("Using clip_marks.json → filtering frames and per-clip references")

        if isinstance(clip_marks, list):
            clip_ranges = [(c["start"], c["end"]) for c in clip_marks]
        elif isinstance(clip_marks, dict):
            clip_ranges = [(v["start"], v["end"]) for v in clip_marks.values()]
        else:
            raise ValueError("clip_marks.json must be a list or dict")

        all_frame_dirs = load_frame_paths(data_dir)
        frame_names = [os.path.basename(f) for f in all_frame_dirs]
        frame_dirs, ref_frames = [], []

        for start, end in clip_ranges:
            try:
                i_start = frame_names.index(start)
                i_end = frame_names.index(end)
                frame_dirs.extend(all_frame_dirs[i_start:i_end + 1])
                ref_frames.append(all_frame_dirs[i_start])
            except ValueError:
                logging.warning(f"Clip range ({start} → {end}) not found, skipping.")
    else:
        frame_dirs = load_frame_paths(data_dir)
        ref_frames = [frame_dirs[0]]
        logging.info("No clip_marks.json found → using all frames and global reference")

    # === Load actions ===
    actions = np.array(load_actions(frame_dirs), dtype=np.float32)

    # === Initialize reference tactile images ===
    ref_dir = ref_frames[0]
    ref_tactile = {
        os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
        for p in glob.glob(os.path.join(ref_dir, "*raw_image.jpg"))
    }

    if use_height_map and ref_tactile:
        logging.info("Using reference tactile images for height map preprocessing")
        index_sensor = init_sensor(
            cfg_path=get_cfg_path("index"),
            calibrated=True,
            ref=ref_tactile.get("rindex_raw_image.jpg"),
            open_camera=False,
        )
        middle_sensor = init_sensor(
            cfg_path=get_cfg_path("middle"),
            calibrated=True,
            ref=ref_tactile.get("rmiddle_raw_image.jpg"),
            open_camera=False,
        )
        thumb_sensor = init_sensor(
            cfg_path=get_cfg_path("thumb"),
            calibrated=True,
            ref=ref_tactile.get("rthumb_raw_image.jpg"),
            open_camera=False,
        )
    elif use_height_map:
        logging.warning("No reference tactile images found; disabling height map preprocessing")
        use_height_map = False

    # === Feature extraction ===
    tactile_list = []
    visual_list = []
    current_ref_dir = ref_dir

    for frame_dir in frame_dirs:
        # --- Update reference if new clip ---
        if frame_dir in ref_frames and frame_dir != current_ref_dir:
            logging.info(f"Updating reference to {frame_dir}")
            ref_tactile = {
                os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
                for p in glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))
            }
            if use_height_map:
                index_sensor.update_ref(ref_tactile.get("rindex_raw_image.jpg"))
                middle_sensor.update_ref(ref_tactile.get("rmiddle_raw_image.jpg"))
                thumb_sensor.update_ref(ref_tactile.get("rthumb_raw_image.jpg"))
            current_ref_dir = frame_dir

        # --- Tactile features ---
        tactile_imgs = glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))
        frame_feats = []

        for img_path in tactile_imgs:
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

            _, _, centroid, major = process_tactile_image(
                img, ref_img=ref, use_height_map=use_height_map, sensor=sensor
            )

            if centroid is None or major is None:
                frame_feats.extend([0.0, 0.0, 0.0, 0.0])
            else:
                frame_feats.extend([centroid[0], centroid[1], major[0], major[1]])

        tactile_list.append(frame_feats)

        # --- Visual embeddings ---
        color1_path = os.path.join(frame_dir, "color_image1.jpg")
        color2_path = os.path.join(frame_dir, "color_image2.jpg")

        color1 = cv2.imread(color1_path)
        color2 = cv2.imread(color2_path)

        emb1 = embedder.embed_rgb(color1) if color1 is not None else None
        emb2 = embedder.embed_rgb(color2) if color2 is not None else None

        if emb1 is None and emb2 is None:
            visual_vec = np.zeros(embedder.out_dim, dtype=np.float32)
        elif emb1 is None:
            visual_vec = emb2.astype(np.float32)
        elif emb2 is None:
            visual_vec = emb1.astype(np.float32)
        else:
            visual_vec = ((emb1 + emb2) / 2.0).astype(np.float32)

        visual_list.append(visual_vec)

    # === Convert to arrays ===
    tactile_feats = np.array(tactile_list, dtype=np.float32)
    visual_feats = np.stack(visual_list, axis=0)
    logging.info(f"Tactile features: {tactile_feats.shape}")
    logging.info(f"Visual features: {visual_feats.shape}")
    logging.info(f"Actions: {actions.shape}")

    joints_t = actions[:-1]
    joints_t1 = actions[1:]
    delta_q = joints_t1 - joints_t
    logging.info(f"joints_t: {joints_t.shape}, joints_t1: {joints_t1.shape}, delta_q: {delta_q.shape}")
    tactile_feats = tactile_feats[:-1]
    visual_feats = visual_feats[:-1]

    # === Save ===
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    np.savez(
        "data/processed/dataset_features.npz",
        tactile_t=np.array(tactile_feats),   # (N, 12)
        visual_t=np.array(visual_feats),     # (N, 256)
        joints_t=np.array(joints_t),            # (N, 23)
        delta_q=np.array(delta_q)           # (N, 23)
    )

    logging.info(f"💾 Saved dataset → {out_file}")

# ---------------- Browsing ----------------
def interactive_browse(data_dir="C:\\Users\\paulm\\franka_ros2_ws\\src\\model_pipeline\\dataset_real_full", use_height_map=False):
    import json

    # --- Handle clip marks if present ---
    clip_marks_path = os.path.join(data_dir, "clip_marks.json")
    if os.path.exists(clip_marks_path):
        with open(clip_marks_path, "r") as f:
            clip_marks = json.load(f)
        logging.info("Using clip_marks.json → filtering frames and per-clip reference")

        if isinstance(clip_marks, list):
            clip_ranges = [(c["start"], c["end"]) for c in clip_marks]
        elif isinstance(clip_marks, dict):
            clip_ranges = [(v["start"], v["end"]) for v in clip_marks.values()]
        else:
            raise ValueError("clip_marks.json must be a list or dict")

        all_frame_dirs = load_frame_paths(data_dir)
        frame_names = [os.path.basename(f) for f in all_frame_dirs]
        frame_dirs = []
        ref_frames = []

        for start, end in clip_ranges:
            try:
                i_start = frame_names.index(start)
                i_end = frame_names.index(end)
                frame_dirs.extend(all_frame_dirs[i_start:i_end + 1])
                ref_frames.append(all_frame_dirs[i_start])
            except ValueError:
                logging.warning(f"Clip range ({start} → {end}) not found, skipping.")
    else:
        frame_dirs = load_frame_paths(data_dir)
        ref_frames = [frame_dirs[0]]
        logging.info("No clip_marks.json found → using all frames and global reference")

    if not frame_dirs:
        logging.error(f"No frames found in {data_dir}")
        return

    # --- Initial reference tactile images ---
    ref_dir = ref_frames[0]
    ref_tactile = {os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
                   for p in glob.glob(os.path.join(ref_dir, "*raw_image.jpg"))}

    if use_height_map and ref_tactile:
        logging.info("Using reference tactile images for preprocessing")
        index_sensor = init_sensor(
            cfg_path=get_cfg_path("index"),
            calibrated=True,
            ref=ref_tactile.get("rindex_raw_image.jpg"),
            open_camera=False)
        middle_sensor = init_sensor(
            cfg_path=get_cfg_path("middle"),
            calibrated=True,
            ref=ref_tactile.get("rmiddle_raw_image.jpg"),
            open_camera=False)
        thumb_sensor = init_sensor(
            cfg_path=get_cfg_path("thumb"),
            calibrated=True,
            ref=ref_tactile.get("rthumb_raw_image.jpg"),
            open_camera=False)
    elif use_height_map:
        logging.warning("No reference tactile images found; cannot use height map preprocessing")
        use_height_map = False

    idx = 0
    current_ref_dir = ref_dir

    while True:
        frame_dir = frame_dirs[idx]

        # If we reached the start of a new clip, update reference
        if frame_dir in ref_frames and frame_dir != current_ref_dir:
            logging.info(f"Updating reference → {frame_dir}")
            ref_tactile = {os.path.basename(p): cv2.imread(p, cv2.IMREAD_UNCHANGED)
                           for p in glob.glob(os.path.join(frame_dir, "*raw_image.jpg"))}
            if use_height_map:
                index_sensor.update_ref(ref_tactile.get("rindex_raw_image.jpg"))
                middle_sensor.update_ref(ref_tactile.get("rmiddle_raw_image.jpg"))
                thumb_sensor.update_ref(ref_tactile.get("rthumb_raw_image.jpg"))
            current_ref_dir = frame_dir

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
            max_width = 1000
            scale = max_width / canvas.shape[1]
            if scale < 1.0:
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            cv2.imshow("Tactile PCA (browse)", canvas)
        else:
            logging.warning(f"No tactile images found in {frame_dir}")

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("n"):
            idx = (idx + 10) % len(frame_dirs)
        elif key == ord("p"):
            idx = (idx - 10) % len(frame_dirs)

    cv2.destroyAllWindows()

    # ---------------- Main ----------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["browse", "export"], default="browse")
    parser.add_argument("--data_dir", type=str, default=str(DEFAULT_DATA_DIR))
    parser.add_argument("--out_file", default="data/processed/dataset_features.npz")
    parser.add_argument("--height_map", action="store_true",
                        help="Use 9DTact height map instead of raw image")
    args = parser.parse_args()

    data_dir = Path(args.data_dir).expanduser().resolve()
    if not data_dir.exists():
        logging.error(f"Data directory not found: {data_dir}")
        return

    if args.mode == "browse":
        interactive_browse(str(data_dir), use_height_map=args.height_map)
    elif args.mode == "export":
        build_npz_dataset(str(data_dir), args.out_file, use_height_map=args.height_map)

if __name__ == "__main__":
    main()