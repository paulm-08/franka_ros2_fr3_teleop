'''
python -m model_pipeline.dataset
python -m model_pipeline.tactile_features --mode export
python -m model_pipeline.split_dataset
python -m model_pipeline.train
python -m model_pipeline.evaluate_policy
python -m model_pipeline.rollout_policy
'''

import os
import cv2
import numpy as np

def create_convoluted_shape(img_size=(200, 200), max_radius=50, num_vertices=6):
    """
    Generate an irregular polygon or deformed ellipse to simulate tactile contact.
    """
    center = np.array([np.random.randint(max_radius, img_size[1]-max_radius),
                       np.random.randint(max_radius, img_size[0]-max_radius)])
    
    angles = np.linspace(0, 2*np.pi, num_vertices, endpoint=False)
    angles += np.random.uniform(-0.3, 0.3, size=num_vertices)  # perturb angles
    radii = np.random.randint(max_radius//2, max_radius, size=num_vertices)
    
    pts = np.zeros((num_vertices, 2), dtype=np.int32)
    pts[:,0] = (center[0] + radii * np.cos(angles)).astype(int)
    pts[:,1] = (center[1] + radii * np.sin(angles)).astype(int)
    
    return pts

def create_dummy_dataset(data_dir="dataset", num_frames=200, img_size=(200, 200)):
    os.makedirs(data_dir, exist_ok=True)

    for i in range(num_frames):
        frame_dir = os.path.join(data_dir, f"frame_{i}")
        os.makedirs(frame_dir, exist_ok=True)

        # Background
        img = np.ones((img_size[0], img_size[1], 3), dtype=np.uint8) * 255

        # Create a convoluted shape
        pts = create_convoluted_shape(img_size=img_size, max_radius=50, num_vertices=np.random.randint(5,12))
        color = tuple(np.random.randint(0, 255, size=3).tolist())
        cv2.fillPoly(img, [pts], color)

        # Apply Gaussian blur to soften edges (simulate gel deformation)
        img = cv2.GaussianBlur(img, (15,15), sigmaX=1, sigmaY=1)

        # Add random Gaussian noise
        noise = np.random.normal(0, 25, img.shape).astype(np.int16)
        noisy_img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Save image
        img_path = os.path.join(frame_dir, "rindex_raw_image.jpg")
        cv2.imwrite(img_path, noisy_img)

        # Save dummy joint data
        joints = np.random.uniform(-1, 1, size=7)
        with open(os.path.join(frame_dir, "right_arm_joint.txt"), "w") as f:
            f.write(" ".join([f"{j:.3f}" for j in joints]))

    print(f"✅ Created complex, noisy dummy dataset in {data_dir}, {num_frames} frames.")


if __name__ == "__main__":
    create_dummy_dataset()
