from ultralytics import YOLO
import cv2
import torch
import glob
import os
import numpy as np
import argparse
from pathlib import Path
import inquirer
import logging

from model_pipeline import paths

# --- Logger Setup ---
logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def find_yolo_models(search_path):
    """
    Recursively searches for 'best.pt' files within the YOLO runs directory.
    Returns a dictionary mapping a display name to its absolute path.
    """
    logging.info(f"Searching for trained YOLO models in: {search_path}...")
    found_models = {}
    for model_path in search_path.rglob("weights/best.pt"):
        run_name = model_path.parent.parent.name
        found_models[run_name] = str(model_path)
    
    logging.info(f"Found {len(found_models)} models.")
    return found_models

def find_demo_dirs(root_search_path):
    """
    Recursively searches for directories that contain 'frame_*' folders.
    """
    logging.info(f"Searching for demonstration directories in: {root_search_path}...")
    found_demos = []
    for dirpath, _, _ in os.walk(root_search_path):
        if glob.glob(os.path.join(dirpath, 'frame_*')):
            relative_path = Path(dirpath).relative_to(paths.WORKSPACE_ROOT)
            found_demos.append(str(relative_path))
            
    logging.info(f"Found {len(found_demos)} potential demonstration directories.")
    return sorted(found_demos)

def draw_detections(image, results, model_names):
    """
    Finds the most confident bounding box for each class and draws it.
    """
    best_boxes = {}
    for box in results.boxes:
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])
        if cls_id not in best_boxes or confidence > best_boxes[cls_id]['confidence']:
            best_boxes[cls_id] = {'box': box.xyxy[0], 'confidence': confidence, 'cls_id': cls_id}

    for data in best_boxes.values():
        x1, y1, x2, y2 = map(int, data['box'])
        confidence = data['confidence']
        class_name = model_names[data['cls_id']]
        label = f'{class_name} {confidence:.2f}'
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    return image

def main():
    parser = argparse.ArgumentParser(description="Interactively test a trained YOLO model on demonstration data.")
    # No arguments needed anymore, as everything is interactive
    args = parser.parse_args()

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    # --- 1. Interactive Model Selection ---
    yolo_runs_dir = paths.YOLO_MODELS_DIR / "detect"
    available_models = find_yolo_models(yolo_runs_dir)

    if not available_models:
        print(f"Error: No 'best.pt' models found in {yolo_runs_dir}. Please train a model first.")
        return

    # --- 2. Interactive Dataset Selection ---
    available_demos = find_demo_dirs(paths.RAW_DATA_DIR)
    if not available_demos:
        print(f"Error: No demonstration directories containing 'frame_*' folders were found in {paths.RAW_DATA_DIR}.")
        return

    # --- Create the interactive questions ---
    questions = [
        inquirer.List('model_name',
                      message="Select the YOLO model run to test",
                      choices=list(available_models.keys()),
                      ),
        inquirer.List('data_dir_rel',
                      message="Select the demonstration dataset to test on",
                      choices=available_demos,
                      ),
    ]

    try:
        answers = inquirer.prompt(questions)
        if not answers:
            print("No selection made. Exiting.")
            return
        selected_model_path = available_models[answers['model_name']]
        # Convert the selected relative path back to an absolute path
        data_path = paths.WORKSPACE_ROOT / answers['data_dir_rel']
    except (KeyboardInterrupt, TypeError):
        print("\nSelection cancelled.")
        return
    
    print(f"Loading model: {selected_model_path}")
    model = YOLO(selected_model_path)
    model.to(device)
    
    if not data_path.is_dir():
        print(f"Error: Data directory not found at {data_path}")
        return

    # --- The rest of the script remains the same ---
    frame_paths = glob.glob(os.path.join(data_path, 'frame_*'))
    frame_dirs = sorted(frame_paths, key=lambda p: int(os.path.basename(p).split('_')[1]))
    
    if not frame_dirs:
        print(f"Error: No 'frame_*' directories found in {data_path}. Check the path.")
        return

    print(f"Found {len(frame_dirs)} frames to process. 'n' for next, 'p' for previous, 'q' to quit.")

    idx = 0
    while True:
        current_frame_dir = frame_dirs[idx]
        frame_name = os.path.basename(current_frame_dir)
        img1_path = os.path.join(current_frame_dir, 'color_image1.jpg')
        img2_path = os.path.join(current_frame_dir, 'color_image2.jpg')

        results = model([img1_path, img2_path], verbose=False, conf=0.1)
        results1, results2 = results[0], results[1]

        img1 = cv2.imread(img1_path) if os.path.exists(img1_path) else np.zeros((480, 640, 3), dtype=np.uint8)
        img2 = cv2.imread(img2_path) if os.path.exists(img2_path) else np.zeros((480, 640, 3), dtype=np.uint8)

        img1_annotated = draw_detections(img1, results1, model.names)
        img2_annotated = draw_detections(img2, results2, model.names)

        combined_img = np.hstack((img1_annotated, img2_annotated))
        
        cv2.putText(combined_img, f'Frame: {frame_name}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('YOLO Side-by-Side Detection', combined_img)

        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'): break
        elif key == ord('n'): idx = (idx + 10) % len(frame_dirs)
        elif key == ord('p'): idx = (idx - 10 + len(frame_dirs)) % len(frame_dirs)

    cv2.destroyAllWindows()
    print("Testing finished.")

if __name__ == '__main__':
    main()

