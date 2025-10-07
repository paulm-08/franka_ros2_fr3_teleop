import os
import numpy as np
import re

def list_frame_dirs(root_dir):
    frame_dirs = [os.path.join(root_dir, d) for d in os.listdir(root_dir) if d.startswith("frame_")]
    # sort numerically
    frame_dirs.sort(key=lambda x: int(re.search(r"frame_(\d+)", x).group(1)))
    return frame_dirs



def load_joint_angles(joint_file):
    if not os.path.exists(joint_file):
        # if missing, return zeros of length 23
        return np.zeros(23, dtype=np.float32)
    arr = np.loadtxt(joint_file)
    return arr.astype(np.float32)