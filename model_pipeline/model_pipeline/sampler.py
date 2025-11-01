import os
import glob
import cv2
import json
import logging
import argparse
from pathlib import Path
import inquirer

from model_pipeline import paths
from model_pipeline.utils import find_demo_dirs

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def sample_frames_by_clip(demo_dirs, output_dir, step):
    """
    (Corrected Version)
    Goes through specified demonstration directories, reads their clip_marks.json,
    and samples one image from each camera every 'step' frames within each clip,
    with robust error checking for file read/write operations.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir = output_dir / "camera_1"
    cam2_dir = output_dir / "camera_2"
    cam1_dir.mkdir(exist_ok=True)
    cam2_dir.mkdir(exist_ok=True)
    
    total_sampled_count = 0
    
    for demo_path_str in demo_dirs:
        demo_path = Path(demo_path_str)
        logging.info(f"\nProcessing directory: {demo_path.name}")
        
        clip_marks_path = demo_path / "clip_marks.json"
        if not clip_marks_path.exists():
            logging.warning(f"  - No clip_marks.json found in {demo_path.name}, skipping.")
            continue

        with open(clip_marks_path, "r") as f:
            clip_marks = json.load(f)
        
        all_frame_dirs = sorted(
            glob.glob(str(demo_path / 'frame_*')), 
            key=lambda p: int(os.path.basename(p).split('_')[1])
        )
        frame_names = [os.path.basename(p) for p in all_frame_dirs]

        clips = clip_marks if isinstance(clip_marks, list) else clip_marks.values()
        for i, clip in enumerate(clips):
            try:
                i_start = frame_names.index(clip["start"])
                i_end = frame_names.index(clip["end"])
                
                logging.info(f"  Sampling from clip {i+1} ({clip['start']} to {clip['end']})...")
                
                for frame_idx in range(i_start, i_end + 1, step):
                    frame_dir = all_frame_dirs[frame_idx]
                    frame_num = os.path.basename(frame_dir).split('_')[1]
                    frame_num_int = int(os.path.basename(frame_dir).split('_')[1])
                    # --- FIX: Zero-pad the frame number to 5 digits (e.g., 3426 -> "03426") ---
                    frame_num_padded = f"{frame_num_int:05d}"


                    # --- Robust sampling for camera 1 ---
                    img1_path = os.path.join(frame_dir, 'color_image1.jpg')
                    if os.path.exists(img1_path):
                        img = cv2.imread(img1_path)
                        if img is None:
                            logging.warning(f"    - Failed to READ image file (it may be corrupted): {img1_path}")
                            continue # Skip this corrupted file
                        
                        # FIX: Added '_frame_' to the filename to match the label format
                        save_name = f"{demo_path.name}_clip_{i}_frame_{frame_num_padded}_cam1.jpg"
                        success = cv2.imwrite(str(cam1_dir / save_name), img)
                        if success:
                            total_sampled_count += 1
                        else:
                            logging.error(f"    - Failed to WRITE image file to: {str(cam1_dir / save_name)}")

                    # --- Robust sampling for camera 2 ---
                    img2_path = os.path.join(frame_dir, 'color_image2.jpg')
                    if os.path.exists(img2_path):
                        img = cv2.imread(img2_path)
                        if img is None:
                            logging.warning(f"    - Failed to READ image file (it may be corrupted): {img2_path}")
                            continue # Skip this corrupted file

                        # FIX: Added '_frame_' to the filename to match the label format
                        save_name = f"{demo_path.name}_clip_{i}_frame_{frame_num_padded}_cam2.jpg"
                        success = cv2.imwrite(str(cam2_dir / save_name), img)
                        if success:
                            total_sampled_count += 1
                        else:
                            logging.error(f"    - Failed to WRITE image file to: {str(cam2_dir / save_name)}")

            except ValueError:
                logging.warning(f"  - Could not find start/end frame for a clip in {demo_path.name}, skipping clip.")

    logging.info(f"\n✅ Done! Sampled a total of {total_sampled_count} images.")
    logging.info(f"Output saved to: {output_dir}")

def main():
    parser = argparse.ArgumentParser(description="Interactively sample images from robot demonstrations for YOLO annotation.")
    parser.add_argument("--output", type=str, default=str(paths.YOLO_DATA_DIR / "sampled_images"), help="Output directory for sampled images.")
    parser.add_argument("--step", type=int, default=30, help="Frame sampling interval.")
    args = parser.parse_args()

    # 1. Automatically find all potential demonstration directories
    demo_choices = find_demo_dirs(paths.RAW_DATA_DIR)
    
    if not demo_choices:
        logging.error(f"No demonstration directories containing 'frame_*' folders were found in {paths.RAW_DATA_DIR}.")
        return

    # 2. Create an interactive checklist for the user
    questions = [
        inquirer.Checkbox('selected_demos',
                          message="Select the datasets to sample from (use SPACE to select, ENTER to confirm)",
                          choices=demo_choices,
                          ),
    ]
    
    try:
        answers = inquirer.prompt(questions)
        if not answers or not answers['selected_demos']:
            logging.info("No datasets selected. Exiting.")
            return
        
        selected_relative_paths = answers['selected_demos']

    except (KeyboardInterrupt, TypeError):
        logging.info("\nSampling cancelled by user.")
        return

    # 3. Convert selected relative paths back to absolute paths
    selected_absolute_paths = [str(paths.WORKSPACE_ROOT / path_str) for path_str in selected_relative_paths]
    
    logging.info(f"You selected {len(selected_absolute_paths)} dataset(s) for sampling.")
    
    # 4. Run the sampling process on the user's selection
    sample_frames_by_clip(selected_absolute_paths, Path(args.output), args.step)

if __name__ == "__main__":
    main()

