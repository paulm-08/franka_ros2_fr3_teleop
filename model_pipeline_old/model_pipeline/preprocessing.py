"""Preprocessing CLI for dataset.

This module scans an input dataset directory structured as:

frame_000/
  color_image1.png
  color_image.png
  depth_image1.png
  depth_image2.png
  right_arm_joint.txt
  rindex_raw_image.jpg
  rmiddle_raw_image.jpg
  rthumb_raw_image.jpg
  action.txt   # optional

It computes tactile PCA features (centroid, angle) per tactile image using a reference frame_0 image and
saves a compact dataset .npz with arrays X (states) and Y (actions).

"""
import os
import argparse
import numpy as np
import cv2
from sklearn.decomposition import PCA
from .dataset_builder import list_frame_dirs, load_joint_angles


def tactile_diff_features(img_path, ref_img_path, lighting_threshold=2, min_pixels=10):
    img = cv2.imread(img_path)
    ref = cv2.imread(ref_img_path)
    if img is None or ref is None:
        raise FileNotFoundError(f"Missing tactile image: {img_path} or {ref_img_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY).astype(np.float32)

    diff = ref_gray - gray
    diff -= lighting_threshold
    diff[diff < 0] = 0
    diff_u8 = np.clip(diff, 0, 255).astype(np.uint8)

    _, mask = cv2.threshold(diff_u8, 50, 255, cv2.THRESH_BINARY)
    coords = np.column_stack(np.where(mask > 0))

    if len(coords) < min_pixels:
        return {'centroid': (0.0, 0.0), 'angle': 0.0, 'contact_pixels': 0}

    centroid = coords.mean(axis=0)
    pca = PCA(n_components=2)
    pca.fit(coords)
    axis = pca.components_[0]
    angle = float(np.arctan2(axis[1], axis[0]))

    return {'centroid': (float(centroid[1]), float(centroid[0])), 'angle': angle, 'contact_pixels': len(coords)}


def process_dataset(root_dir, out_path, tactile_names=None):
    if tactile_names is None:
        tactile_names = ['rindex_raw_image.jpg', 'rmiddle_raw_image.jpg', 'rthumb_raw_image.jpg']

    frame_dirs = list_frame_dirs(root_dir)
    if len(frame_dirs) == 0:
        raise RuntimeError('No frame_* directories found')

    # use frame_0 as reference for all tactile sensors
    ref_dir = frame_dirs[0]
    ref_paths = {name: os.path.join(ref_dir, name) for name in tactile_names}

    X = []
    Y = []

    for frame_dir in frame_dirs:
        # load joints
        joints_file = os.path.join(frame_dir, 'right_arm_joint.txt')
        joints = load_joint_angles(joints_file)

        tactile_feats = []
        for name in tactile_names:
            img_path = os.path.join(frame_dir, name)
            ref_path = ref_paths[name]
            feats = tactile_diff_features(img_path, ref_path)
            # centroid_x, centroid_y, angle, contact_pixels
            tactile_feats.extend([feats['centroid'][0], feats['centroid'][1], feats['angle'], feats['contact_pixels']])

        # TODO: add nozzle position resolution here (if available). For now placeholder zeros
        nozzle_pos = [0.0, 0.0, 0.0]

        # action
        action_file = os.path.join(frame_dir, 'action.txt')
        if os.path.exists(action_file):
            action = np.loadtxt(action_file)
        else:
            # placeholder: zeros (user should provide actions or convert teleop logs)
            action = np.zeros(6)

        state = np.concatenate([nozzle_pos, np.array(tactile_feats), joints])
        X.append(state)
        Y.append(action)

    X = np.array(X)
    Y = np.array(Y)
    np.savez_compressed(out_path, X=X, Y=Y)
    print(f"Saved processed dataset to {out_path}.npz with shapes X={X.shape}, Y={Y.shape}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', '-d', required=True, help='Path to dataset root containing frame_* folders')
    parser.add_argument('--out', '-o', required=True, help='Output npz path (without extension)')
    args = parser.parse_args()
    process_dataset(args.dataset, args.out)


if __name__ == '__main__':
    main()