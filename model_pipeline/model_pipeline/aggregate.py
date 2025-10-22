import os
import shutil
from pathlib import Path
import argparse
import logging
import inquirer

from model_pipeline import paths

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def find_annotated_dirs(root_search_path):
    """
    Recursively searches a root directory to find all subdirectories that
    contain annotation (.txt) files.
    """
    logging.info(f"Searching for annotation directories in: {root_search_path}...")
    found_dirs = []
    for dirpath, _, filenames in os.walk(root_search_path):
        if any(fname.endswith('.txt') for fname in filenames):
            relative_path = Path(dirpath).relative_to(paths.WORKSPACE_ROOT)
            found_dirs.append(str(relative_path))
            
    logging.info(f"Found {len(found_dirs)} potential annotation directories.")
    return sorted(found_dirs)

def find_image_dirs(root_search_path):
    """
    Recursively searches a root directory to find all subdirectories that
    contain image (.jpg, .png) files.
    """
    logging.info(f"Searching for image directories in: {root_search_path}...")
    found_dirs = []
    image_extensions = {'.jpg', '.jpeg', '.png'}
    for dirpath, _, filenames in os.walk(root_search_path):
        if any(Path(fname).suffix.lower() in image_extensions for fname in filenames):
            relative_path = Path(dirpath).relative_to(paths.WORKSPACE_ROOT)
            found_dirs.append(str(relative_path))
            
    logging.info(f"Found {len(found_dirs)} potential image directories.")
    return sorted(found_dirs)

def aggregate_data(source_label_dirs, source_image_dirs, dest_dir):
    """
    For each label file in the source directories, finds the matching image
    across all selected image directories and copies both to the destination.
    """
    dest_path = Path(dest_dir)
    images_dest = dest_path / "images"
    labels_dest = dest_path / "labels"
    
    images_dest.mkdir(parents=True, exist_ok=True)
    labels_dest.mkdir(parents=True, exist_ok=True)
    
    img_count = 0
    label_count = 0
    unmatched_labels = 0
    
    # --- Build a map of all available images for fast lookup ---
    logging.info("Building an index of all available images from selected source(s)...")
    image_map = {}
    for image_dir_str in source_image_dirs:
        image_path = Path(image_dir_str)
        for image_file in image_path.rglob('*'):
            if image_file.suffix.lower() in ['.jpg', '.jpeg', '.png']:
                if image_file.stem in image_map:
                    logging.warning(f"  - Duplicate image name found: '{image_file.name}'. The first one found will be used.")
                else:
                    image_map[image_file.stem] = image_file
    logging.info(f"Indexed {len(image_map)} unique images.")
    
    logging.info(f"Aggregating data into '{dest_path}'...")

    for source_dir_str in source_label_dirs:
        source_path = Path(source_dir_str)
        if not source_path.is_dir():
            logging.warning(f"Source '{source_dir_str}' is not a valid directory. Skipping.")
            continue
            
        logging.info(f"  -> Processing annotations in '{source_path.name}'")
        for label_file in source_path.glob('*.txt'):
            image_stem = label_file.stem
            
            # Use the pre-built map for a fast lookup
            image_file = image_map.get(image_stem)

            if image_file:
                # Copy both the label and the image
                shutil.copy(label_file, labels_dest / label_file.name)
                shutil.copy(image_file, images_dest / image_file.name)
                img_count += 1
                label_count += 1
            else:
                logging.warning(f"    - No matching image found for label: {label_file.name}")
                unmatched_labels += 1

    logging.info(f"✅ Aggregation complete.")
    logging.info(f"   Copied {img_count} images to '{images_dest}'")
    logging.info(f"   Copied {label_count} labels to '{labels_dest}'")
    if unmatched_labels > 0:
        logging.warning(f"   Could not find matching images for {unmatched_labels} labels.")

def main():
    parser = argparse.ArgumentParser(
        description="Interactively find and aggregate labeled YOLO data from multiple sources."
    )
    parser.add_argument('--dest', '-d', type=str, 
                        default=str(paths.YOLO_DATA_DIR / "aggregated_dataset"), 
                        help='Destination directory to copy all files into')
    args = parser.parse_args()

    # 1. Automatically find all potential directories for images and labels
    annotated_dir_choices = find_annotated_dirs(paths.YOLO_DATA_DIR)
    image_dir_choices = find_image_dirs(paths.YOLO_DATA_DIR)
    
    if not annotated_dir_choices:
        logging.error(f"No directories containing annotation (.txt) files were found in {paths.YOLO_DATA_DIR}.")
        return
    if not image_dir_choices:
        logging.error(f"No directories containing image (.jpg, .png) files were found in {paths.YOLO_DATA_DIR}.")
        return

    # 2. Create two separate interactive questions
    questions = [
        inquirer.Checkbox('selected_labels',
                          message="Select the ANNOTATION directories to aggregate (SPACE to select, ENTER)",
                          choices=annotated_dir_choices),
        inquirer.Checkbox('selected_images',
                          message="Select the IMAGE directories to search within (SPACE to select, ENTER)",
                          choices=image_dir_choices),
    ]
    try:
        answers = inquirer.prompt(questions)
        if not answers or not answers.get('selected_labels') or not answers.get('selected_images'):
            logging.info("No label or image directories selected. Exiting."); return
        selected_label_paths = answers['selected_labels']
        selected_image_paths = answers['selected_images']
    except (KeyboardInterrupt, TypeError):
        logging.info("\nAggregation cancelled by user."); return

    # 3. Ask for confirmation to clear the destination directory
    dest_path = Path(args.dest)
    if dest_path.exists():
        clear_question = [
            inquirer.Confirm('clear',
                             message=f"Destination '{dest_path.name}' already exists. Clear it before aggregating?",
                             default=True),
        ]
        try:
            if inquirer.prompt(clear_question)['clear']:
                logging.info(f"Clearing destination directory: {dest_path}")
                shutil.rmtree(dest_path)
        except (KeyboardInterrupt, TypeError):
            logging.info("\nAggregation cancelled by user."); return

    # 4. Run the aggregation process
    selected_label_abs_paths = [str(paths.WORKSPACE_ROOT / path_str) for path_str in selected_label_paths]
    selected_image_abs_paths = [str(paths.WORKSPACE_ROOT / path_str) for path_str in selected_image_paths]
    
    logging.info(f"Aggregating from {len(selected_label_abs_paths)} annotation source(s).")
    logging.info(f"Searching for images in {len(selected_image_abs_paths)} source(s).")
    aggregate_data(selected_label_abs_paths, selected_image_abs_paths, args.dest)

if __name__ == "__main__":
    main()
