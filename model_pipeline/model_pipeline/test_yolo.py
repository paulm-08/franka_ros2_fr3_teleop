# test_yolo.py (Draw Most Confident Box Per Class)
from ultralytics import YOLO
import cv2
import torch
import glob
import os
import numpy as np

# --- Configuration ---
MODEL_PATH = r'C:\Users\paulm\franka_ros2_ws\runs\detect\yolov8_custom6\weights\best.pt'
DATA_DIR = r'data/recorded_data/tube5'

def draw_detections(image, results, model_names):
    """
    Helper function to find the most confident bounding box for each class
    and draw only those boxes on the image.
    """
    # --- START OF MODIFICATION ---
    best_boxes = {} # Dictionary to store the best box for each class ID

    for box in results.boxes:
        cls_id = int(box.cls[0])
        confidence = float(box.conf[0])

        # If this class is not in our dict, or if the current box is more confident, store it
        if cls_id not in best_boxes or confidence > best_boxes[cls_id]['confidence']:
            best_boxes[cls_id] = {
                'box': box.xyxy[0],
                'confidence': confidence,
                'cls_id': cls_id
            }

    # Now, draw only the boxes stored in our dictionary
    for data in best_boxes.values():
        x1, y1, x2, y2 = map(int, data['box'])
        confidence = data['confidence']
        class_name = model_names[data['cls_id']]
        label = f'{class_name} {confidence:.2f}'

        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
    # --- END OF MODIFICATION ---
    return image

def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Using device: {device}")

    model = YOLO(MODEL_PATH)
    model.to(device)

    frame_paths = glob.glob(os.path.join(DATA_DIR, 'frame_*'))
    frame_dirs = sorted(frame_paths, key=lambda p: int(os.path.basename(p).split('_')[1]))
    
    if not frame_dirs:
        print(f"Error: No 'frame_*' directories found in {DATA_DIR}. Check the path.")
        return

    print(f"Found {len(frame_dirs)} frames to process. 'n' for next, 'p' for previous, 'q' to quit.")

    idx = 0
    while True:
        current_frame_dir = frame_dirs[idx]
        frame_name = os.path.basename(current_frame_dir)

        img1_path = os.path.join(current_frame_dir, 'color_image1.jpg')
        img2_path = os.path.join(current_frame_dir, 'color_image2.jpg')

        img1 = cv2.imread(img1_path) if os.path.exists(img1_path) else np.zeros((480, 640, 3), dtype=np.uint8)
        img2 = cv2.imread(img2_path) if os.path.exists(img2_path) else np.zeros((480, 640, 3), dtype=np.uint8)

        # Lower the confidence threshold to see weak detections
        results = model([img1_path, img2_path], verbose=False, conf=0.005)
        results1, results2 = results[0], results[1]

        img1_annotated = draw_detections(img1, results1, model.names)
        img2_annotated = draw_detections(img2, results2, model.names)

        combined_img = np.hstack((img1_annotated, img2_annotated))

        print(f"Frame: {frame_name} | Detections - Image1: {len(results1.boxes)}, Image2: {len(results2.boxes)}")

        cv2.putText(combined_img, f'Frame: {frame_name}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('YOLO Side-by-Side Detection', combined_img)

        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            idx = (idx + 10) % len(frame_dirs)
        elif key == ord('p'):
            idx = (idx - 10 + len(frame_dirs)) % len(frame_dirs)

    cv2.destroyAllWindows()
    print("Testing finished.")

if __name__ == '__main__':
    main()