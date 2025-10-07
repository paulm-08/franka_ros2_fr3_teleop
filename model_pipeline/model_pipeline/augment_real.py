import os
import cv2
import numpy as np
from glob import glob

def load_image(path):
    return cv2.imread(path, cv2.IMREAD_UNCHANGED)

def save_image(path, img):
    cv2.imwrite(path, img)

def interpolate_array(a, b, alpha):
    return (1 - alpha) * a + alpha * b

def interpolate_images(img1, img2, alpha, perturb=False):
    blended = interpolate_array(img1.astype(np.float32), img2.astype(np.float32), alpha)
    if perturb:
        noise = np.random.normal(0, 2, blended.shape).astype(np.float32)
        blended = np.clip(blended + noise, 0, 255)
    return blended.astype(img1.dtype)

def interpolate_joints(j1, j2, alpha, perturb=False):
    arr1 = np.array(j1)
    arr2 = np.array(j2)
    blended = interpolate_array(arr1, arr2, alpha)
    if perturb:
        blended += np.random.normal(0, 0.01, size=blended.shape)  # small joint noise
    return blended

def parse_joints(path):
    with open(path, "r") as f:
        return np.array([float(x) for x in f.read().split()])

def save_joints(path, joints):
    with open(path, "w") as f:
        f.write(" ".join([f"{j:.3f}" for j in joints]))

def augment_dataset(real_dir="C:\\Users\\paulm\\franka_ros2_ws\\src\\model_pipeline\\dataset_real", out_dir="C:\\Users\\paulm\\franka_ros2_ws\\src\\model_pipeline\\dataset_real_full"):
    os.makedirs(out_dir, exist_ok=True)

    frames = sorted(glob(os.path.join(real_dir, "frame_*")))
    frame_indices = [int(os.path.basename(f).split("_")[1]) for f in frames]
    print(f"Found keyframes: {frame_indices}")

    # Load all keyframes into memory
    keyframes = {idx: f for idx, f in zip(frame_indices, frames)}

    for i in range(len(frame_indices) - 1):
        start_idx, end_idx = frame_indices[i], frame_indices[i + 1]
        frame_a, frame_b = keyframes[start_idx], keyframes[end_idx]

        # Load joints once
        joints_a = parse_joints(os.path.join(frame_a, "right_arm_joint.txt"))
        joints_b = parse_joints(os.path.join(frame_b, "right_arm_joint.txt"))

        for t in range(start_idx, end_idx + 1):
            alpha = (t - start_idx) / (end_idx - start_idx) if end_idx > start_idx else 0
            new_frame = os.path.join(out_dir, f"frame_{t}")
            os.makedirs(new_frame, exist_ok=True)

            for fname in ["color_image1.jpg", "color_image2.jpg",
                          "depth_image1.png", "depth_image2.png",
                          "rindex_raw_image.jpg", "rmiddle_raw_image.jpg", "rthumb_raw_image.jpg"]:
                img_a = load_image(os.path.join(frame_a, fname))
                img_b = load_image(os.path.join(frame_b, fname))

                # tactile images: perturb interpolation
                perturb = "raw_image" in fname
                img_interp = interpolate_images(img_a, img_b, alpha, perturb=perturb)
                save_image(os.path.join(new_frame, fname), img_interp)

            # joints
            joints_interp = interpolate_joints(joints_a, joints_b, alpha, perturb=True)
            save_joints(os.path.join(new_frame, "right_arm_joint.txt"), joints_interp)

    print(f"✅ Full dataset written to {out_dir} with {frame_indices[-1]+1} frames.")

if __name__ == "__main__":
    augment_dataset()
