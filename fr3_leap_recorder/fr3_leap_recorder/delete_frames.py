#!/usr/bin/env python3
import os
import re
import argparse
import shutil

def delete_frames(dataset_dir, between=None, dry_run=False):
    frame_pattern = re.compile(r"^frame_(\d+)$")

    frames = []
    for name in os.listdir(dataset_dir):
        match = frame_pattern.match(name)
        if match:
            frame_id = int(match.group(1))
            frames.append((frame_id, name))

    if not frames:
        print("No frame directories found.")
        return

    frames.sort(key=lambda x: x[0])
    deleted = []

    # Parse between range
    min_id, max_id = None, None
    if between:
        if len(between) == 1:
            max_id = between[0]
        elif len(between) == 2:
            min_id, max_id = between
        else:
            raise ValueError("`--between` must have 1 or 2 integers")

    for frame_id, frame_name in frames:
        delete = False

        if min_id is not None and max_id is not None:
            if min_id <= frame_id <= max_id:
                delete = True
        elif max_id is not None:
            if frame_id <= max_id:
                delete = True

        if delete:
            frame_path = os.path.join(dataset_dir, frame_name)
            if dry_run:
                print(f"[DRY RUN] Would delete {frame_path}")
            else:
                shutil.rmtree(frame_path)
                print(f"Deleted {frame_path}")
            deleted.append(frame_id)

    if not deleted:
        print("No frames matched deletion criteria.")
    else:
        print(f"Deleted {len(deleted)} frames.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Delete frame directories by suffix number.")
    parser.add_argument("dataset_dir", help="Path to dataset directory")
    parser.add_argument("--between", nargs="+", type=int, help="One value: delete ≤ N. Two values: delete between N and M inclusive.")
    parser.add_argument("--dry-run", action="store_true", help="List deletions without removing")

    args = parser.parse_args()

    delete_frames(
        args.dataset_dir,
        between=args.between,
        dry_run=args.dry_run
    )
