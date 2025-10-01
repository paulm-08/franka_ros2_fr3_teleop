import os
import cv2
import numpy as np
from sklearn.decomposition import PCA

# ----------------------------
# Tactile preprocessing
# ----------------------------
def tactile_features(img_path):
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)  # contact mask

    coords = np.column_stack(np.where(mask > 0))
    if len(coords) < 10:
        return [0, 0, 0]  # no contact detected

    # centroid
    centroid = coords.mean(axis=0)  # (v, u)

    # PCA orientation
    pca = PCA(n_components=2)
    pca.fit(coords)
    angle = np.arctan2(pca.components_[0, 1], pca.components_[0, 0])

    return [centroid[1], centroid[0], angle]  # (u, v, θ)


# ----------------------------
# Example: build state vector
# ----------------------------
def build_state(frame_dir):
    # Vision (stub) -------------------
    # load RGB + depth
    rgb1 = cv2.imread(os.path.join(frame_dir, "color_image1.jpg"))
    rgb2 = cv2.imread(os.path.join(frame_dir, "color_image2.jpg"))
    depth1 = cv2.imread(os.path.join(frame_dir, "depth_image1.png"), cv2.IMREAD_UNCHANGED)
    depth2 = cv2.imread(os.path.join(frame_dir, "depth_image2.png"), cv2.IMREAD_UNCHANGED)

    # TODO: here you need nozzle detection → returns (x,y,z) in robot frame
    # for now, just put placeholder (0,0,0)
    nozzle_pos = np.zeros(3)

    # Tactile -------------------------
    tactile_feats = []
    for fname in ["rindex_raw_image.jpg", "rmiddle_raw_image.jpg", "rthumb_raw_image.jpg"]:
        tactile_feats.extend(tactile_features(os.path.join(frame_dir, fname)))

    # Joints --------------------------
    joint_file = os.path.join(frame_dir, "right_arm_joint.txt")
    joint_angles = np.loadtxt(joint_file)  # shape (23,)
    joint_feats = joint_angles.tolist()

    # State vector --------------------
    state = np.concatenate([nozzle_pos, tactile_feats, joint_feats])
    return state

# ----------------------------
# Dataset builder
# ----------------------------
def build_dataset(root_dir):
    X = []
    Y = []

    frame_dirs = sorted([os.path.join(root_dir, d) for d in os.listdir(root_dir) if d.startswith("frame_")])
    i=0
    for frame_dir in frame_dirs :  # limit to first 100 frames for testing
        if i<100:
            i+=1
            state = build_state(frame_dir)

            # Load action: depends on how you stored it
            # If action = Δpose or velocities are in a file "action.txt"
            action_file = os.path.join(frame_dir, "action.txt")
            if os.path.exists(action_file):
                action = np.loadtxt(action_file)
            else:
                action = np.zeros(6)  # placeholder [Δx,Δy,Δz,Δroll,Δpitch,Δyaw]

            X.append(state)
            Y.append(action)
        else:
            break

    X = np.array(X)
    Y = np.array(Y)
    return X, Y

dataset_root = "/home/user/recorded_data/clipped_data/tube4"
X, Y = build_dataset(dataset_root)

print("Dataset shapes:", X.shape, Y.shape)
# X ~ (N_frames, ~35 features)
# Y ~ (N_frames, action_dim)
