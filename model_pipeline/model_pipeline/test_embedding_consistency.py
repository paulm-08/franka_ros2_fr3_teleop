import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
import numpy as np
import cv2
import os
import logging
import sys
from pathlib import Path
import pyrealsense2 as rs 
import time

# --- Setup Logging ---
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# --- CONFIGURATION ---
DEBUG_DIR = Path("./embedding_test_comparison_live")
SAVED_JPEG_PATH = DEBUG_DIR / "test_color_image.jpg"
SAVED_PNG_PATH = DEBUG_DIR / "test_color_image.png"

LIVE_DEGRADED_PATH = DEBUG_DIR / "live_debug_degraded_pre_transform.png"
LIVE_CLEAN_PATH = DEBUG_DIR / "live_debug_clean_pre_transform.png"
DATASET_JPEG_PATH = DEBUG_DIR / "dataset_debug_jpeg_pre_transform.png"
DATASET_PNG_PATH = DEBUG_DIR / "dataset_debug_png_pre_transform.png"

# --- NEW EXTERNAL CONFIGURATION (CRITICAL) ---
# IMPORTANT: Update this path to point to a real JPEG image file from your *training dataset*!
EXTERNAL_DATASET_JPEG_PATH = Path("/home/user/franka_ros2_ws/data/recorded_data/clipped_data/tube13/frame_139/color_image1.jpg")
EXTERNAL_DEBUG_PATH = DEBUG_DIR / "external_dataset_debug_pre_transform.png"
# -----------------------------------

# --- CHECKPOINT CONFIGURATION (NEW) ---
CHECKPOINT_PATH = Path("/home/user/franka_ros2_ws/models/policy_models/policy_mlp_joint_space_arm15.pt") 
# IMPORTANT: This key MUST match where the VFE state_dict is stored in your .pt file.
# If your VFE is saved under a different key in the checkpoint, ADJUST THIS.
# Examples: 'vfe_embedder', 'policy.visual_embedder', 'state_dict' (if only VFE is saved)
VFE_STATE_DICT_KEY = 'vfe_embedder' 
# ----------------------------------------

REALSENSE_SERIAL = '151422254571'

# --- 1. MINIMAL VISUAL EMBEDDER (Updated to match user's architecture) ---
class VisualEmbedder(nn.Module):
    def __init__(
        self,
        backbone: str = "resnet18",
        device: str | None = None,
        pretrained: bool = True,
        out_dim: dict[str, int] | None = None,
        global_depth_range: tuple[float, float] | None = None,
    ):
        super().__init__()
        if device is None:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = torch.device(device)
        self.backbone_name = backbone

        if backbone == "resnet18":
            weights = models.ResNet18_Weights.DEFAULT if pretrained else None
            base_model = models.resnet18(weights=weights)
            self.feat_dim = 512
            modules = list(base_model.children())[:-1]
            self.encoder = nn.Sequential(*modules)
            self.post_flatten = lambda t: t.reshape(t.shape[0], -1)
        else:
            # Note: MobileNetV3 support is omitted here for simplicity as ResNet18 is the default
            raise ValueError(f"Unsupported backbone: {backbone}")

        if out_dim is None:
            # Using the user's default dimension from their VisionProcessor code
            out_dim = {"rgb": 256, "depth": 128} 
        self.out_dim = out_dim
        self.project_rgb = nn.Linear(self.feat_dim, self.out_dim["rgb"])
        self.project_depth = nn.Linear(self.feat_dim, self.out_dim["depth"])

        if global_depth_range:
            self.global_min_depth, self.global_max_depth = global_depth_range
        else:
            self.global_min_depth, self.global_max_depth = 0.0, 1.0

        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            ),
        ])

        self.to(self.device)
        self.eval()
        torch.set_grad_enabled(False)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        logging.info(f"VisualEmbedder on '{self.device}' set to eval mode.")

    @torch.no_grad()
    def embed_rgb(self, img_bgr: np.ndarray, debug_save_path: Path = None) -> np.ndarray | None:
        """Embed a single BGR image."""
        if img_bgr is None:
            return None
        
        # Convert BGR (OpenCV/RealSense format) to RGB (PyTorch/Pillow format)
        # Note: The user's code suggests their input might already be RGB, but we must
        # rely on the standard CV2 conversion for robustness in this test setup.
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        if debug_save_path:
            # Save the *RGB* image that is about to be transformed
            cv2.imwrite(str(debug_save_path), cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
            logging.info(f"Saved pre-transform debug image to {debug_save_path.name}")

        t = self.transform(img_rgb).unsqueeze(0).to(self.device)
        feats = self.encoder(t)
        feats = self.post_flatten(feats)
        projected_feats = self.project_rgb(feats)
        return projected_feats.squeeze(0).cpu().numpy()

# --- NEW UTILITY FUNCTION TO LOAD FINE-TUNED WEIGHTS ---
def load_vfe_weights(embedder: VisualEmbedder, checkpoint_path: Path, vfe_state_dict_key: str):
    """Loads the Visual Embedder's fine-tuned weights from the policy checkpoint."""
    if not checkpoint_path.is_file():
        logging.error(f"Checkpoint not found at: {checkpoint_path}. VFE weights NOT loaded.")
        return False
        
    try:
        logging.info(f"Attempting to load VFE weights from checkpoint: {checkpoint_path.name}")
        checkpoint = torch.load(checkpoint_path, map_location=embedder.device)
        
        # 1. Try to find the state_dict directly
        state_dict = checkpoint.get(vfe_state_dict_key)

        # 2. If the user's checkpoint is the full policy, we need to extract the VFE keys.
        if state_dict is None:
            logging.warning(f"Did not find state_dict at key '{vfe_state_dict_key}'. Assuming full policy checkpoint.")
            
            # Filter the policy's state_dict for keys starting with 'visual_embedder.'
            # Note: This prefix might vary (e.g., 'policy.vfe.', 'vfe.'). User might need to adjust.
            vfe_prefix = 'visual_embedder.' 
            state_dict = {
                k.replace(vfe_prefix, ''): v for k, v in checkpoint['state_dict'].items() if k.startswith(vfe_prefix)
            }
            if not state_dict:
                logging.error(f"Could not find VFE weights using key '{vfe_state_dict_key}' or prefix '{vfe_prefix}' in checkpoint.")
                return False

        # Load the extracted state dict into the VFE model
        embedder.load_state_dict(state_dict, strict=True)
        logging.info("✅ Successfully loaded fine-tuned VFE weights from checkpoint.")
        return True

    except Exception as e:
        logging.error(f"FATAL ERROR loading VFE weights from checkpoint: {e}")
        logging.error("Check VFE_STATE_DICT_KEY and checkpoint file structure.")
        return False

# --- 2. SIMULATION FUNCTIONS (unchanged) ---

def simulate_recording():
    """
    Initializes a Realsense camera, captures a single BGR frame,
    and saves it to disk (JPEG and PNG) to simulate the dataset creation process.
    """
    logging.info("--- Simulating Recording (with RealSense Live Frame) ---")
    
    pipeline = rs.pipeline()
    config = rs.config()
    live_bgr_frame = None
    
    try:
        # Check if the desired camera is connected
        ctx = rs.context()
        devices = ctx.query_devices()
        found = False
        for dev in devices:
            if dev.get_info(rs.camera_info.serial_number) == REALSENSE_SERIAL:
                found = True
                break
        if not found:
            logging.error(f"FATAL: Camera {REALSENSE_SERIAL} not found. Please check config.")
            return None

        # Configure and start the stream (640x480 BGR at 30fps)
        config.enable_device(REALSENSE_SERIAL)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        logging.info(f"Started camera {REALSENSE_SERIAL}...")

        # Wait for auto-exposure/gain to settle (30 frames)
        logging.info("Waiting for auto-exposure...")
        for _ in range(30):
            pipeline.wait_for_frames()
        
        # Get the definitive frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            logging.error("Failed to get color frame.")
            return None
            
        # This is the "live" BGR frame, straight from the camera (in-memory)
        live_bgr_frame = np.asanyarray(color_frame.get_data())
        
        # 1. Save it to disk with JPEG (Lossy, simulating dataset)
        cv2.imwrite(str(SAVED_JPEG_PATH), live_bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 95]) # Quality 95
        logging.info(f"Saved live frame to {SAVED_JPEG_PATH.name} (JPEG Dataset Sim)")
        
        # 2. Save it to disk with PNG (Lossless, for the "clean" reference)
        cv2.imwrite(str(SAVED_PNG_PATH), live_bgr_frame)
        logging.info(f"Saved live frame to {SAVED_PNG_PATH.name} (PNG Clean Ref)")
        
        return live_bgr_frame

    except Exception as e:
        logging.error(f"Realsense error: {e}")
        return None
    finally:
        pipeline.stop()
        logging.info("Camera stopped.")


def simulate_dataset_loading(embedder: VisualEmbedder, image_path: Path, debug_image_path: Path):
    """Loads the saved image (like the dataset builder) and gets its embedding."""
    loaded_bgr_image = cv2.imread(str(image_path))
    if loaded_bgr_image is None:
        logging.error(f"Failed to read image: {image_path.name}")
        return None
        
    logging.info(f"Loaded {image_path.name} (dtype: {loaded_bgr_image.dtype})")
    embedding = embedder.embed_rgb(loaded_bgr_image, debug_save_path=debug_image_path)
    return embedding

def simulate_live_rollout_clean(embedder: VisualEmbedder, live_bgr_frame: np.ndarray, debug_image_path: Path):
    """Uses the 'live' BGR array (from memory) without degradation."""
    logging.info("--- Simulating Live Rollout (CLEAN) ---")
    embedding = embedder.embed_rgb(live_bgr_frame.astype(np.uint8), debug_save_path=debug_image_path)
    return embedding

def simulate_live_rollout_with_degradation(embedder: VisualEmbedder, live_bgr_frame: np.ndarray, debug_image_path: Path):
    """
    Uses the 'live' BGR array (from memory) AFTER applying JPEG degradation (the workaround).
    """
    logging.info("--- Simulating Live Rollout (DEGRADED - THE WORKAROUND) ---")
    
    # --- DEGRADATION WORKAROUND ---
    # 1. Encode to JPEG (lossy compression) using the same quality (95)
    success, encoded_image = cv2.imencode('.jpg', live_bgr_frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    
    if not success:
        logging.error("JPEG encoding FAILED!")
        return None
        
    # 2. Decode it back (introduces artifacts, returns BGR)
    degraded_bgr_frame = cv2.imdecode(encoded_image, cv2.IMREAD_COLOR)
    
    if degraded_bgr_frame is None:
        logging.error("JPEG decoding FAILED! Returned None.")
        return None
    
    embedding = embedder.embed_rgb(degraded_bgr_frame, debug_save_path=debug_image_path)
    return embedding

def run_comparison(name: str, ref_embedding: np.ndarray, current_embedding: np.ndarray, ref_img_path: Path, current_img_path: Path):
    """Helper function to run comparisons."""
    print("-" * 40)
    logging.info(f"--- COMPARISON: {name} ---")
    
    if current_embedding is None:
        logging.error(f"Comparison failed: {name} embedding is None.")
        return

    # Calculate Difference
    abs_diff = np.abs(ref_embedding - current_embedding)
    mean_abs_diff = np.mean(abs_diff)
    max_abs_diff = np.max(abs_diff)

    logging.info(f"Embedding Dimension: {current_embedding.shape[0]}")
    logging.info(f"Mean Absolute Difference: {mean_abs_diff:.6f}")
    logging.info(f"Max Absolute Difference:  {max_abs_diff:.6f}")

    # Image Check (Sanity)
    try:
        ref_img = cv2.imread(str(ref_img_path))
        current_img = cv2.imread(str(current_img_path))
        if ref_img is None or current_img is None:
            logging.error("Could not load debug images for comparison.")
            return
            
        # Calculate pixel difference (for visual artifacts check)
        image_diff = np.sum(np.abs(ref_img.astype(float) - current_img.astype(float)))
        logging.info(f"Pre-transform Image Diff (Sum of Abs Diff): {image_diff}")
        
        if image_diff < 1e-9: 
            logging.info("✅ Pre-transform debug images are PIXEL-IDENTICAL (Lossless only).")
        else:
            logging.warning("⚠️ Pre-transform debug images are DIFFERENT (Expected for lossy/degraded).")
            
    except Exception as e:
        sys.stderr.write(f"CRITICAL IMAGE COMPARE FAILURE: {e}\n")
        logging.error(f"Could not compare debug images: {e}")

# --- 3. MAIN EXECUTION (Updated to load weights) ---

def run_test():
    DEBUG_DIR.mkdir(exist_ok=True)
    
    # 1. Init Embedder (Loads default ImageNet weights first)
    embedder = VisualEmbedder(out_dim={"rgb": 256, "depth": 128})
    
    # 1B. CRITICAL STEP: Load the fine-tuned weights from the policy checkpoint
    # This aligns the live VFE weights with the ones used to create the normalized features (X_mean, X_std)
    load_vfe_weights(embedder, CHECKPOINT_PATH, VFE_STATE_DICT_KEY)

    # 2. Simulate Recording (ACQUIRE LIVE FRAME AND SAVE JPEG/PNG)
    live_bgr_frame_from_cam = simulate_recording()
    if live_bgr_frame_from_cam is None:
        return 
    
    # --- GET ALL 4 EMBEDDINGS ---
    
    # 3. Baseline 1: Lossless Dataset (PNG) - The "True" feature set
    dataset_png_embedding = simulate_dataset_loading(embedder, SAVED_PNG_PATH, DATASET_PNG_PATH)
    
    # 4. Baseline 2: Problematic Dataset (JPEG) - What your model was trained on
    dataset_jpeg_embedding = simulate_dataset_loading(embedder, SAVED_JPEG_PATH, DATASET_JPEG_PATH)
    
    # 5. Live Input 1: Clean Rollout (Original setup) - Should differ from JPEG Baseline
    rollout_clean_embedding = simulate_live_rollout_clean(embedder, live_bgr_frame_from_cam, LIVE_CLEAN_PATH)
    
    # 6. Live Input 2: Degraded Rollout (Workaround setup) - Should match JPEG Baseline
    rollout_degraded_embedding = simulate_live_rollout_with_degradation(embedder, live_bgr_frame_from_cam, LIVE_DEGRADED_PATH)

    print("=" * 40)
    logging.info("--- FINAL COMPARISON RESULTS ---")
    if any(e is None for e in [dataset_png_embedding, dataset_jpeg_embedding, rollout_clean_embedding, rollout_degraded_embedding]):
        logging.error("Test failed, one or more necessary embeddings are None. Cannot compare.")
        return

    # A. PNG Dataset vs. JPEG Dataset: How much does JPEG loss hurt the features?
    run_comparison(
        "PNG_DATASET vs. JPEG_DATASET (Quantifies Artifact Impact on features)", 
        dataset_png_embedding, 
        dataset_jpeg_embedding, 
        DATASET_PNG_PATH, 
        DATASET_JPEG_PATH
    )

    # B. JPEG Dataset vs. Clean Live: THE ORIGINAL ISSUE (Should show a significant difference)
    run_comparison(
        "JPEG_DATASET vs. CLEAN_LIVE (Original Failure Scenario - Large expected diff)", 
        dataset_jpeg_embedding, 
        rollout_clean_embedding, 
        DATASET_JPEG_PATH, 
        LIVE_CLEAN_PATH
    )

    # C. JPEG Dataset vs. Degraded Live: THE WORKAROUND TEST (This MUST be near zero)
    run_comparison(
        "JPEG_DATASET vs. DEGRADED_LIVE (The Workaround Test - Expected diff < 1e-6)", 
        dataset_jpeg_embedding, 
        rollout_degraded_embedding, 
        DATASET_JPEG_PATH, 
        LIVE_DEGRADED_PATH
    )
    
    # --- D. NEW: EXTERNAL DATASET COMPARISON (FEATURE DISTANCE CHECK) ---
    external_path = EXTERNAL_DATASET_JPEG_PATH
    if external_path.is_file():
        logging.info("\n" + "*" * 50)
        logging.info("--- RUNNING EXTERNAL DATASET COMPARISON ---")
        logging.info(f"Loading external file: {external_path.name}")
        
        external_dataset_embedding = simulate_dataset_loading(
            embedder, 
            external_path, 
            EXTERNAL_DEBUG_PATH
        )
        
        if external_dataset_embedding is not None and rollout_degraded_embedding is not None:
            
            # --- CRITICAL COMPARISON D1: Dataset Image vs. Degraded Live Image (Different Scenes) ---
            # This comparison shows the feature difference between two different scenes,
            # using the SAME (live) VFE. The difference should be "large" but still plausible.
            abs_diff_inter_scene = np.abs(external_dataset_embedding - rollout_degraded_embedding)
            mean_abs_diff_inter_scene = np.mean(abs_diff_inter_scene)

            run_comparison(
                "EXTERNAL_DATASET_JPEG vs. DEGRADED_LIVE (Feature Distance Check on Different Scenes)", 
                external_dataset_embedding, 
                rollout_degraded_embedding, 
                EXTERNAL_DEBUG_PATH, 
                LIVE_DEGRADED_PATH
            )

            # --- CRITICAL COMPARISON D2: Dataset Image vs. Degraded Live Image (Same Scene) ---
            # This comparison isolates the VFE output difference on two different images.
            run_comparison(
                "EXTERNAL_DATASET_JPEG vs. JPEG_DATASET_SIM (Feature Distance Check - VFE output on two different images)", 
                external_dataset_embedding, 
                dataset_jpeg_embedding, # This uses the live VFE on the simulated JPEG image
                EXTERNAL_DEBUG_PATH, 
                DATASET_JPEG_PATH
            )
            
            logging.info("--- Interpretation of Distance (Comparison D1) ---")
            logging.info(f"The inter-scene distance (Mean Abs Diff: {mean_abs_diff_inter_scene:.6f}) is the feature dissimilarity between the live frame and the external dataset image, both processed by the CURRENT live VFE.")
            
            # Since the scenes are different, we check if the distance is in a plausible range
            # We use a relatively high threshold (0.5) to determine "plausible similarity"
            if mean_abs_diff_inter_scene < 0.5: 
                logging.info(f"✅ The distance ({mean_abs_diff_inter_scene:.4f}) suggests the degraded live input is within a plausible feature range for a similar-but-different image (using the current VFE).")
            else:
                logging.warning(f"⚠️ The distance ({mean_abs_diff_inter_scene:.4f}) is quite large. This might indicate the live scene is too dissimilar to the dataset, or a non-matching compression profile.")
                
            logging.info("*" * 50)
            
        else:
            logging.error("Failed to load or embed external dataset JPEG. Skipping ultimate comparison.")
            
    else:
        logging.warning("\n" + "*" * 50)
        logging.warning(f"External comparison skipped: File not found at {external_path}. You MUST update EXTERNAL_DATASET_JPEG_PATH in the script.")
        logging.warning("*" * 50)
        
    print("=" * 40)
    # Final check on the standard workaround success (JPEG Sim vs Degraded Live)
    abs_diff_match = np.abs(dataset_jpeg_embedding - rollout_degraded_embedding)
    mean_abs_diff_match = np.mean(abs_diff_match)

    if mean_abs_diff_match < 1e-6:
        logging.info(f"🎉 SUCCESS: Workaround Features Match Simulated Dataset Features! Mean Diff: {mean_abs_diff_match:.8e}")
        logging.info("➡️ The logic for the degradation workaround is confirmed correct by this test (Simulated JPEG).")
    else:
        logging.error(f"❌ FAILURE: Workaround Features DO NOT match Simulated Dataset Features. Mean Diff: {mean_abs_diff_match:.8e}")
        logging.error("➡️ Something is fundamentally different between the live degradation and the dataset image loading.")


if __name__ == "__main__":
    run_test()