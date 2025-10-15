import os
import random
import shutil
from pathlib import Path
import argparse
import logging

from model_pipeline import paths # Import the new paths module

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def split_data(images_dir, labels_dir, dest_dir, split_ratio=0.8):
    """
    Splits image and annotation data from separate source folders into
    training and validation sets for YOLO.

    Args:
        images_dir (str): Source directory containing the image files (.jpg, .png).
        labels_dir (str): Source directory containing the annotation files (.txt).
        dest_dir (str): Root directory to create the 'train' and 'valid' subfolders.
        split_ratio (float): The proportion of data to use for training.
    """
    images_path = Path(images_dir)
    labels_path = Path(labels_dir)
    dest_path = Path(dest_dir)

    # --- Input Validation ---
    if not images_path.is_dir() or not labels_path.is_dir():
        logging.error("Image and Label source paths must be existing directories.")
        return
    if not 0 < split_ratio < 1:
        logging.error("Split ratio must be between 0 and 1.")
        return

    # --- Find all image files ---
    # Supports multiple common image formats
    image_extensions = ["*.jpg", "*.jpeg", "*.png"]
    img_files = []
    for ext in image_extensions:
        img_files.extend(list(images_path.glob(ext)))

    if not img_files:
        logging.error(f"No image files found in '{images_dir}'.")
        return

    logging.info(f"Found {len(img_files)} image files in '{images_dir}'.")
    random.shuffle(img_files)

    # --- Create destination directories ---
    train_img_path = dest_path / "images" / "train"
    train_lbl_path = dest_path / "labels" / "train"
    valid_img_path = dest_path / "images" / "valid"
    valid_lbl_path = dest_path / "labels" / "valid"
    for p in [train_img_path, train_lbl_path, valid_img_path, valid_lbl_path]:
        p.mkdir(parents=True, exist_ok=True)

    # --- Split files and define move function ---
    split_index = int(len(img_files) * split_ratio)
    train_files = img_files[:split_index]
    valid_files = img_files[split_index:]

    def save_files(file_list, img_dest, lbl_dest):
        """Moves an image and its corresponding label file."""
        moved_count = 0
        for img_path in file_list:
            # Construct the expected path for the corresponding label file
            lbl_filename = img_path.stem + ".txt"
            lbl_source_path = labels_path / lbl_filename

            if lbl_source_path.exists():
                shutil.copy(str(img_path), str(img_dest))
                shutil.copy(str(lbl_source_path), str(lbl_dest))
                moved_count += 1
            else:
                logging.warning(f"Annotation '{lbl_filename}' not found for image '{img_path.name}'. Skipping this file.")
        return moved_count

    # --- Execute the move ---
    logging.info("Splitting files...")
    num_train = save_files(train_files, train_img_path, train_lbl_path)
    num_valid = save_files(valid_files, valid_img_path, valid_lbl_path)

    logging.info(f"✅ Done! Split dataset into '{dest_path}'.")
    logging.info(f"   Training set: {num_train} image-label pairs.")
    logging.info(f"   Validation set: {num_valid} image-label pairs.")

def main():
    parser = argparse.ArgumentParser(description="Split aggregated image and label data for YOLO training.")
    # The script now takes a single source directory and assumes it contains 'images' and 'labels' subfolders
    parser.add_argument("--source", type=str, default=str(paths.YOLO_DATA_DIR / "aggregated_dataset"),
                        help="Source directory with 'images' and 'labels' subfolders.")
    parser.add_argument("--dest", type=str, default=str(paths.YOLO_DATA_DIR / "yolo_split_dataset"), 
                        help="Destination directory for the train/valid split.")
    parser.add_argument("--ratio", type=float, default=0.8, help="Training set split ratio.")
    args = parser.parse_args()

    source_path = Path(args.source)
    images_path = source_path / "images"
    labels_path = source_path / "labels"

    if not images_path.exists() or not labels_path.exists():
        logging.error(f"Error: Source directory '{source_path}' must contain 'images' and 'labels' subfolders.")
        return

    split_data(str(images_path), str(labels_path), args.dest, args.ratio)

if __name__ == "__main__":
    main()
