import matplotlib.pyplot as plt
import numpy as np
import cv2
from sklearn.decomposition import PCA

def show_tactile_with_pca(img_path, ref_img_path):
    # Load images
    img = cv2.imread(img_path)
    ref_img = cv2.imread(ref_img_path)
    img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ref_GRAY = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)

    # Difference image (reference - current)
    diff_raw = ref_GRAY.astype(np.float32) - img_GRAY.astype(np.float32)

    plt.figure(figsize=(12,6))
    plt.subplot(1,3,1)
    plt.title("Current tactile image")
    plt.imshow(img[..., ::-1])  # show RGB version of tactile image
    plt.axis('off')
    plt.subplot(1,3,2)
    plt.title("Reference tactile image")
    plt.imshow(ref_img[..., ::-1])  # show RGB version of tactile image
    plt.axis('off')
    plt.subplot(1,3,3)
    plt.title("Difference image")
    plt.imshow(diff_raw, cmap="gray")
    plt.axis('off')

    # Thresholding to remove small lighting differences
    lighting_threshold = 2
    diff_raw -= lighting_threshold
    diff_raw[diff_raw < 0] = 0

    # Binarize contact region
    _, mask = cv2.threshold(diff_raw.astype(np.uint8), 2, 255, cv2.THRESH_BINARY)
    plt.figure(figsize=(6,6))
    plt.title("Contact mask")
    plt.imshow(mask, cmap="gray")
    plt.axis('off')

    coords = np.column_stack(np.where(mask > 0))

    if len(coords) < 10:
        print("No contact detected")
        plt.imshow(img[..., ::-1])  # show RGB version of tactile image
        return None, None

    # Centroid of contact
    centroid = coords.mean(axis=0)  # (v, u)

    # PCA for orientation
    pca = PCA(n_components=2)
    pca.fit(coords)
    axis = pca.components_[0]
    angle = np.arctan2(axis[1], axis[0])

    # Visualization
    plt.figure(figsize=(6,6))
    plt.imshow(diff_raw, cmap="gray")
    plt.scatter(centroid[1], centroid[0], c="red", marker="x", label="centroid")

    # Draw PCA axis
    length = 50
    x0, y0 = centroid[1], centroid[0]
    dx, dy = axis[0]*length, axis[1]*length
    plt.plot([x0-dx, x0+dx], [y0-dy, y0+dy], "r-", linewidth=2, label="PCA axis")

    plt.legend()
    plt.title(f"PCA angle = {np.degrees(angle):.1f}°")
    plt.axis('off')
    plt.show()

    return centroid, angle

centroid, angle = show_tactile_with_pca(
    img_path="/home/user/recorded_data/clipped_data/tube4/frame_500/rindex_raw_image.jpg",
    ref_img_path="/home/user/recorded_data/clipped_data/tube4/frame_0/rindex_raw_image.jpg"
)

print("Centroid:", centroid, "Angle (rad):", angle)
