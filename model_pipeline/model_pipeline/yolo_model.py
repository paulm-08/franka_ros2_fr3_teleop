from ultralytics import YOLO

# This function will now contain your main logic
def main():
    # Load a pre-trained model
    model = YOLO('yolov8n.pt')

    # Train on custom dataset
    model.train(
        data='data/data.yaml',
        imgsz=640,
        epochs=50,
        batch=8,
        name='yolov8_custom'
    )

if __name__ == '__main__':
    main()