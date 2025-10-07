import argparse
import os
import cv2
import numpy as np
import yaml
from sklearn.decomposition import PCA
from ament_index_python.packages import get_package_share_directory

from .dataset_builder import list_frame_dirs
from tact9d.shape_reconstruction.sensor import Sensor


class TactilePCA:
    def __init__(self, ref=None):
        """Load configs for thumb, index, middle sensors."""
        base_path = os.path.join(
            get_package_share_directory("tact9d"), "shape_reconstruction"
        )

        def load_sensor(name):
            path = os.path.join(base_path, f"shape_config_{name}.yaml")
            if not os.path.exists(path):
                raise FileNotFoundError(f"Missing config: {path}")
            with open(path, "r", encoding="utf-8") as f:
                cfg = yaml.load(f, Loader=yaml.FullLoader)
            return Sensor(cfg, ref=ref)

        self.sensors = {
            "rthumb_raw_image.jpg": load_sensor("thumb"),
            "rindex_raw_image.jpg": load_sensor("index"),
            "rmiddle_raw_image.jpg": load_sensor("middle"),
        }

    def compute_pca_overlay(self, img_path, ref_path, show_on="diff"):
        """Compute PCA overlay on diff, raw, or mask view."""
        img = cv2.imread(img_path)
        ref = cv2.imread(ref_path)
        if img is None or ref is None:
            print(f"[WARN] Could not read {img_path} or {ref_path}")
            return np.zeros((128, 128, 3), np.uint8), None, None

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)

        # Difference image
        diff = ref_gray.astype(np.float32) - gray.astype(np.float32)
        diff = np.clip(diff - 2, 0, None)  # subtract lighting bias

        _, mask = cv2.threshold(diff.astype(np.uint8), 3, 255, cv2.THRESH_BINARY)
        coords = np.column_stack(np.where(mask > 0))

        # Choose visualization background
        if show_on == "diff":
            base = cv2.normalize(diff, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            shown = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
        elif show_on == "mask":
            shown = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            shown = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX)
            shown = cv2.cvtColor(shown.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        # PCA and overlay
        if len(coords) > 10:
            centroid = coords.mean(axis=0)  # (y, x)
            pca = PCA(n_components=2).fit(coords)
            axis = pca.components_[0]
            angle = np.arctan2(axis[1], axis[0])

            cx, cy = int(centroid[1]), int(centroid[0])
            cv2.drawMarker(shown, (cx, cy), (0, 0, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
            dx, dy = int(axis[0]*50), int(axis[1]*50)
            cv2.line(shown, (cx-dx, cy-dy), (cx+dx, cy+dy), (0, 0, 255), 2)

            return shown, centroid, angle
        else:
            return shown, None, None

    def show_frame(self, frame_dir, ref_dir, tactile_names, window="Tactile PCA", mode="raw"):
        """Render tactile images side-by-side with PCA overlays."""
        imgs, results = [], {}
        for name in tactile_names:
            img_path = os.path.join(frame_dir, name)
            ref_path = os.path.join(ref_dir, name)
            shown, centroid, angle = self.compute_pca_overlay(img_path, ref_path, show_on=mode)
            cv2.putText(shown, name, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
            imgs.append(shown)
            results[name] = (centroid, angle)

        stacked = cv2.hconcat(imgs)
        cv2.imshow(window, stacked)
        return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dataset", required=True)
    parser.add_argument("--mode", choices=["raw", "diff", "mask"], default="raw")
    args = parser.parse_args()

    frame_dirs = list_frame_dirs(args.dataset)
    if not frame_dirs:
        print("No frames found in dataset")
        return

    tactile_names = ["rindex_raw_image.jpg",
                     "rmiddle_raw_image.jpg",
                     "rthumb_raw_image.jpg"]

    ref_dir, pos = frame_dirs[0], 0
    pca = TactilePCA()

    while True:
        print(f"Frame {pos}/{len(frame_dirs)-1}: {frame_dirs[pos]}")
        pca.show_frame(frame_dirs[pos], ref_dir, tactile_names, mode=args.mode)

        key = cv2.waitKey(0) & 0xFF
        if key in (ord('q'), 27):  # quit
            break
        elif key in (ord('n'), ord('6')):
            pos = min(pos+1, len(frame_dirs)-1)
        elif key in (ord('p'), ord('4')):
            pos = max(pos-1, 0)
        else:
            print("Keys: n/6=next, p/4=prev, q=quit")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
