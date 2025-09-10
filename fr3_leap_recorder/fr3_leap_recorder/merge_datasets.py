import os
import re
import shutil
from natsort import natsorted

# -------------------------------
# CONFIGURATION
# -------------------------------
BASE_DIR = "/home/user/recorded_data"  # Change this if needed, otherwise use current directory

# Regex to capture datasets like "test", "test2", but NOT "testA"
DATASET_PATTERN = re.compile(r"^(.*?)(\d*)$")


def get_dataset_groups(base_dir):
    """Group datasets by their base name if suffix is empty or purely numeric."""
    dirs = [d for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    dataset_groups = {}

    for d in dirs:
        match = DATASET_PATTERN.match(d)
        if not match:
            continue

        base, suffix = match.groups()

        # Skip directories with non-numeric suffixes
        if suffix and not suffix.isdigit():
            continue

        dataset_groups.setdefault(base, []).append((d, int(suffix) if suffix else 0))

    # Sort each group by numeric suffix
    for base in dataset_groups:
        dataset_groups[base] = sorted(dataset_groups[base], key=lambda x: x[1])

    return dataset_groups


def merge_datasets(base_dir):
    dataset_groups = get_dataset_groups(base_dir)

    for base, dirs_with_suffix in dataset_groups.items():
        if len(dirs_with_suffix) < 2:
            continue  # Nothing to merge

        print(f"\nMerging datasets for base name: {base}")

        # First directory becomes the main merged one
        # main_dir = os.path.join(base_dir, dirs_with_suffix[0][0])
        main_dir = os.path.join(base_dir, base)
        if not os.path.exists(main_dir):
            os.rename(os.path.join(base_dir, dirs_with_suffix[0][0]), main_dir)

        # Get current last frame index in main_dir
        frame_dirs = [
            d for d in os.listdir(main_dir)
            if os.path.isdir(os.path.join(main_dir, d)) and d.startswith("frame_")
        ]
        frame_dirs = natsorted(frame_dirs)
        last_frame_id = (
            int(frame_dirs[-1].split("_")[1]) if frame_dirs else -1
        )
        next_frame_id = last_frame_id + 1

        # Merge remaining dirs into main_dir
        for sub_dir, _ in dirs_with_suffix[1:]:
            sub_path = os.path.join(base_dir, sub_dir)
            sub_frames = [
                d for d in os.listdir(sub_path)
                if os.path.isdir(os.path.join(sub_path, d)) and d.startswith("frame_")
            ]
            sub_frames = natsorted(sub_frames)

            for frame in sub_frames:
                src = os.path.join(sub_path, frame)
                dst = os.path.join(main_dir, f"frame_{next_frame_id}")
                shutil.move(src, dst)
                next_frame_id += 1

            # Remove the now-empty dataset folder
            shutil.rmtree(sub_path)

        print(f" -> Merged {len(dirs_with_suffix)} datasets into '{main_dir}'")
        print(f" -> Final frame count: {next_frame_id} frames")


if __name__ == "__main__":
    merge_datasets(BASE_DIR)
    print("\nAll merges complete ✅")
