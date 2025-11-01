#!/usr/bin/env python3
import os, re, json, sys, glob
from os import path
import yaml
import cv2
import numpy as np

# temperary hard-coded default config path
# DEFAULT_CONFIG = "/home/user/ViHaTeleop-main/ViHaTeleop/configs/data_processing/clip_config_sim.yaml"
DEFAULT_CONFIG = "/home/user/ViHaTeleop-main/ViHaTeleop/configs/data_processing/clip_config.yaml"


def load_config(cfg_path: str):
    with open(cfg_path, "r") as f:
        return yaml.safe_load(f)

def find_frames(dataset_root, frame_regex="^frame_(\\d+)$"):
    rx = re.compile(frame_regex)
    frames = []
    for d in os.listdir(dataset_root):
        full = path.join(dataset_root, d)
        if path.isdir(full):
            m = rx.match(d)
            if m:
                idx = int(m.group(1))
                frames.append((idx, d, full))
    frames.sort(key=lambda x: x[0])
    return frames

def detect_cam_ids(frame_dir, color_pattern):
    """
    Auto-detect camera indices by scanning for files matching the color pattern.
    color_pattern has a '{cam}' placeholder, e.g., 'color_image{cam}.jpg'
    We look for files like color_image1.jpg, color_image02.jpg, etc.
    """
    # Build a regex from the pattern
    # Escape everything except the {cam} placeholder -> capture one or more digits
    esc = re.escape(color_pattern).replace("\\{cam\\}", r"(\d+)")
    rx = re.compile("^" + esc + "$")
    ids = []
    for f in os.listdir(frame_dir):
        m = rx.match(f)
        if m:
            ids.append(int(m.group(1)))
    ids = sorted(set(ids))
    return ids

def read_color(frame_dir, color_pattern, cam_id):
    fname = color_pattern.replace("{cam}", str(cam_id))
    fpath = path.join(frame_dir, fname)
    if path.exists(fpath):
        img = cv2.imread(fpath, cv2.IMREAD_COLOR)
        return img
    return None

def read_depth(frame_dir, depth_pattern, cam_id):
    fname = depth_pattern.replace("{cam}", str(cam_id))
    fpath = path.join(frame_dir, fname)
    if path.exists(fpath):
        dep = cv2.imread(fpath, cv2.IMREAD_UNCHANGED)
        return dep
    return None

def read_tactile(frame_dir, fname):
    fpath = path.join(frame_dir, fname)
    if path.exists(fpath):
        img = cv2.imread(fpath, cv2.IMREAD_GRAYSCALE)
        return img
    return None

def tactile_diff(img, ref, cmap=cv2.COLORMAP_JET):
    if img is None or ref is None:
        return None
    diff = cv2.absdiff(img, ref)
    diff_norm = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX)
    diff_color = cv2.applyColorMap(diff_norm, cmap)
    return diff_color


def to_heatmap(depth_raw, normalize_per_image=True, global_minmax=None, cmap_name="JET"):
    """
    Convert depth to a colored heatmap. Zero/invalid depths shown as black.
    """
    if depth_raw is None:
        return None

    # Pick a colormap
    cmap_map = {
        "JET": cv2.COLORMAP_JET,
        "TURBO": cv2.COLORMAP_TURBO,
        "INFERNO": cv2.COLORMAP_INFERNO
    }
    cmap = cmap_map.get(cmap_name.upper(), cv2.COLORMAP_JET)

    depth = depth_raw.astype(np.float32)

    # Mask invalid (<=0)
    valid = depth > 0
    if not np.any(valid):
        # return black image
        return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

    if normalize_per_image or global_minmax is None:
        dmin = float(depth[valid].min())
        dmax = float(depth[valid].max())
    else:
        dmin, dmax = global_minmax

    if dmax <= dmin:
        dmax = dmin + 1.0

    norm = np.clip((depth - dmin) / (dmax - dmin), 0.0, 1.0)
    norm_u8 = (norm * 255).astype(np.uint8)

    norm_u8 = cv2.equalizeHist(norm_u8)

    heat = cv2.applyColorMap(norm_u8, cmap)
    # paint invalid pixels black
    heat[~valid] = 0
    return heat

def label_image(img, text):
    if img is None:
        return None
    out = img.copy()
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(out, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)
    return out

def montage_for_frame(frame_dir, cam_ids, color_pattern, depth_pattern,
                      depth_heatmap="JET", normalize_depth_per_image=True,
                      max_preview_width=1920, tactile_patterns=None, ref_tactile=None):
    """
    For each cam: [COLOR | DEPTH HEATMAP]
    Stack all cams vertically.
    """
    rows = []
    panel_w = None

    # Cameras
    for cid in cam_ids:
        color = read_color(frame_dir, color_pattern, cid)
        depth = read_depth(frame_dir, depth_pattern, cid)
        color_labeled = label_image(color, f"cam{cid} color") if color is not None else None
        heat = to_heatmap(depth, normalize_per_image=normalize_depth_per_image,
                          global_minmax=None, cmap_name=depth_heatmap)
        heat_labeled = label_image(heat, f"cam{cid} depth") if heat is not None else None

        # If neither exists, skip this camera
        if color_labeled is None and heat_labeled is None:
            continue

        # If one modality missing, create a blank placeholder to keep layout consistent
        if color_labeled is None and heat_labeled is not None:
            h, w = heat_labeled.shape[:2]
            color_labeled = np.zeros((h, w, 3), dtype=np.uint8)
        if heat_labeled is None and color_labeled is not None:
            h, w = color_labeled.shape[:2]
            heat_labeled = np.zeros((h, w, 3), dtype=np.uint8)
        row = np.hstack([color_labeled, heat_labeled])
        rows.append(row)
        panel_w = row.shape[1]

    # Tactile
    if tactile_patterns and ref_tactile:
        tactile_row = []
        for pat in tactile_patterns:
            img = read_tactile(frame_dir, pat)
            diff = tactile_diff(img, ref_tactile.get(pat))
            if diff is None:
                continue
            tactile_row.append(label_image(diff, pat.replace("_raw_image.jpg", "")))

        if tactile_row:
            row = np.hstack(tactile_row)

            # 🔧 Ensure same width as panel_w
            if panel_w is not None and row.shape[1] != panel_w:
                row = cv2.resize(row, (panel_w, row.shape[0]), interpolation=cv2.INTER_AREA)

            rows.append(row)

    if not rows:
        return None

    canvas = np.vstack(rows)

    # Resize to max width if needed
    if max_preview_width and canvas.shape[1] > max_preview_width:
        scale = max_preview_width / canvas.shape[1]
        new_w = max_preview_width
        new_h = int(canvas.shape[0] * scale)
        canvas = cv2.resize(canvas, (new_w, new_h), interpolation=cv2.INTER_AREA)

    return canvas

def overlay_status(canvas, lines):
    if canvas is None:
        canvas = 255 * np.ones((480, 640, 3), dtype=np.uint8)
    y = 30
    out = canvas.copy()
    for line in lines:
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 1, cv2.LINE_AA)
        y += 30
    return out

def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CONFIG
    if not path.exists(cfg_path):
        print(f"Config file not found: {cfg_path}")
        sys.exit(1)

    cfg = load_config(cfg_path)
    dataset_root   = cfg["dataset_root"]
    frame_regex    = cfg.get("frame_folder_regex", r"^frame_(\d+)$")
    color_pattern  = cfg.get("color_pattern", "color_image{cam}.jpg")
    depth_pattern  = cfg.get("depth_pattern", "depth_image{cam}.png")
    preview        = bool(cfg.get("preview", True))
    skip_empty     = bool(cfg.get("skip_empty_frames", True))
    auto_detect    = bool(cfg.get("auto_detect_cams", True))
    cam_ids        = cfg.get("cam_ids")  # may be None
    depth_heatmap  = cfg.get("depth_heatmap", "JET")
    norm_per_img   = bool(cfg.get("normalize_depth_per_image", True))
    max_width      = int(cfg.get("max_preview_width", 1920))
    tactile_patterns = cfg.get("tactile_patterns", [])

    assert path.isdir(dataset_root), f"dataset_root not found: {dataset_root}"

    frames = find_frames(dataset_root, frame_regex)
    if not frames:
        print("No frames found. Check dataset_root and frame_folder_regex.")
        sys.exit(1)

    ref_tactile = {}
    if tactile_patterns:
        first_frame_dir = frames[0][2]
        for pat in tactile_patterns:
            ref_tactile[pat] = read_tactile(first_frame_dir, pat)

    # Determine camera IDs
    if cam_ids is None and auto_detect:
        # Use first frame with content to detect cams
        for _, fname, fdir in frames:
            cam_ids = detect_cam_ids(fdir, color_pattern)
            if cam_ids:
                break
        if not cam_ids:
            print("Auto-detect failed: no color files found in first frames. "
                  "Set cam_ids explicitly in config.")
            sys.exit(1)
    elif cam_ids is None:
        # if auto_detect_cams=false but no cam_ids, default to [1]
        cam_ids = [1]

    # Optionally filter out frames with no preview images (any cam color OR depth present counts)
    if skip_empty:
        kept = []
        for (i, fname, fdir) in frames:
            has_any = False
            for cid in cam_ids:
                if path.exists(path.join(fdir, color_pattern.replace("{cam}", str(cid)))) \
                   or path.exists(path.join(fdir, depth_pattern.replace("{cam}", str(cid)))):
                    has_any = True
                    break
            if has_any:
                kept.append((i, fname, fdir))
        frames = kept
        if not frames:
            print("All frames filtered out (no images found). Disable skip_empty_frames if desired.")
            sys.exit(1)

    pos = 0
    clip_marks = []
    current_clip = {}
    window_name = "Clipping (s=start, e=end, 4/-10, 6/+10, 0=save, q=quit)"

    if preview:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    def save_now():
        out_path = path.join(dataset_root, "clip_marks.json")
        with open(out_path, "w") as f:
            json.dump(clip_marks, f, indent=2)
        print(f"Saved {out_path} with {len(clip_marks)} clip(s).")

    jump_step = int(cfg.get("jump_step", 10))  # default = 10

    while True:
        idx, fname, fdir = frames[pos]

        canvas = montage_for_frame(
            fdir, cam_ids,
            color_pattern=color_pattern,
            depth_pattern=depth_pattern,
            depth_heatmap=depth_heatmap,
            normalize_depth_per_image=norm_per_img,
            max_preview_width=max_width,
            tactile_patterns=tactile_patterns,
            ref_tactile=ref_tactile
        )

        status = [f"Frame: {fname}  ({pos+1}/{len(frames)})",
                  f"Cameras: {cam_ids}",
                  "Keys: s=start, e=end, 4/-10, 6/+10, 0=save, q=quit"]
        if "start" in current_clip:
            status.insert(1, f"Current clip START = {current_clip['start']} (waiting for END)")

        shown = overlay_status(canvas, status) if preview else None

        if preview:
            cv2.imshow(window_name, shown)
            key = cv2.waitKey(0) & 0xFF
        else:
            print("\n".join(status))
            key = ord(sys.stdin.read(1))

        if key in (ord('q'), 27):  # q or Esc
            if "start" in current_clip:
                print("Dropping incomplete clip:", current_clip["start"])
                current_clip = {}
            save_now()
            break

        elif key == ord('0'):
            save_now()

        elif key == ord('s'):
            current_clip = {"start": fname}
            print("Start =", fname)

        elif key == ord('e'):
            if "start" not in current_clip:
                print("No active start. Press 's' first.")
            else:
                start, end = current_clip["start"], fname
                # ensure numeric ordering
                getn = lambda s: int(re.search(r"(\d+)", s).group(1))
                if getn(end) < getn(start):
                    start, end = end, start
                clip_marks.append({"start": start, "end": end})
                print("Added clip:", clip_marks[-1])
                current_clip = {}

        elif key == ord('4'):  # back
            pos = max(0, pos - jump_step)
        elif key == ord('6'):  # forward
            pos = min(len(frames)-1, pos + jump_step)
        else:
            print("Unknown key.")

    if preview:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
