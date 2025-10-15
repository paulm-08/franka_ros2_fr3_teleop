import os
import json
import shutil
from pathlib import Path
import argparse
import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def sample_frames_by_clip(demo_dirs, output_dir, step=30):
    """
    Samples frames from within each clip for both cameras and saves them into
    separate directories with zero-padded frame numbers for correct sorting.
    """
    output_path = Path(output_dir)
    cam1_path = output_path / "cam1_images"
    cam2_path = output_path / "cam2_images"
    cam1_path.mkdir(parents=True, exist_ok=True)
    cam2_path.mkdir(parents=True, exist_ok=True)

    img_count = 0

    for demo_dir in demo_dirs:
        demo_path = Path(demo_dir)
        demo_name = demo_path.name
        clip_marks_path = demo_path / "clip_marks.json"

        if not clip_marks_path.exists():
            logging.warning(f"No clip_marks.json in {demo_dir}, skipping.")
            continue

        logging.info(f"Processing clips for demonstration: {demo_name}")
        with open(clip_marks_path, "r") as f:
            clips = json.load(f)

        # **FIX**: Sort frame paths numerically instead of alphabetically.
        all_frame_paths = sorted(
            demo_path.glob("frame_*"),
            key=lambda p: int(p.name.split('_')[1])
        )
        all_frame_names = [p.name for p in all_frame_paths]

        clip_iterable = None
        if isinstance(clips, dict):
            clip_iterable = clips.items()
        elif isinstance(clips, list):
            clip_iterable = ((clip.get("name", f"clip_{i}"), clip) for i, clip in enumerate(clips))
        else:
            logging.error(f"Unsupported format for clip_marks.json in {demo_dir}. Must be a dict or list.")
            continue

        for clip_name, clip_data in clip_iterable:
            start_frame, end_frame = clip_data["start"], clip_data["end"]
            
            try:
                i_start = all_frame_names.index(start_frame)
                i_end = all_frame_names.index(end_frame)
            except ValueError:
                logging.error(f"Frame range for clip '{clip_name}' not found. Skipping.")
                continue

            clip_frame_paths = all_frame_paths[i_start : i_end + 1]

            for i in range(0, len(clip_frame_paths), step):
                frame_path = clip_frame_paths[i]

                # --- START: Loop for Both Cameras ---
                for cam_idx in [1, 2]:
                    src_img_path = frame_path / f"color_image{cam_idx}.jpg"
                    if src_img_path.exists():
                        # **FIX**: Extract frame number and zero-pad it for correct sorting.
                        frame_num = int(frame_path.name.split('_')[1])
                        padded_frame_name = f"frame_{frame_num:05d}" # e.g., frame_00353

                        # Create the new, informative filename.
                        unique_name = f"{demo_name}_{clip_name}_{padded_frame_name}_cam{cam_idx}.jpg"
                        
                        # Select the correct destination folder based on the camera.
                        if cam_idx == 1:
                            dest_img_path = cam1_path / unique_name
                        else: # cam_idx == 2
                            dest_img_path = cam2_path / unique_name
                        
                        shutil.copy(src_img_path, dest_img_path)
                        img_count += 1
                # --- END: Loop for Both Cameras ---

    logging.info(f"✅ Done! Sampled {img_count} images into separate camera folders in '{output_path}'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sample images from robot demonstrations by clip for two cameras.")
    parser.add_argument("--demos", nargs="+", required=True, help="List of demonstration directories.")
    parser.add_argument("--output", type=str, required=True, help="Output directory for sampled images.")
    parser.add_argument("--step", type=int, default=30, help="Frame sampling interval.")
    args = parser.parse_args()
    
    sample_frames_by_clip(args.demos, args.output, args.step)