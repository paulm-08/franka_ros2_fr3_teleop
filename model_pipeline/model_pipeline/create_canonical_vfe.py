import torch
import torch.nn as nn
import torchvision.models as models
from pathlib import Path
import logging
from model_pipeline.visual_embedder import VisualEmbedder

def main():
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    
    # Define the output path (e.g., in your models directory)
    # PLEASE UPDATE THIS PATH
    SAVE_PATH = Path("/home/user/franka_ros2_ws/models/vfe_canonical_resnet18_128d.pt")
    
    if SAVE_PATH.exists():
        logging.warning(f"Weight file already exists at: {SAVE_PATH}")
        logging.warning("Skipping. Delete the file if you want to regenerate it.")
    else:
        logging.info("Creating new canonical VFE weights...")
        # We set pretrained=True to load ImageNet
        vfe = VisualEmbedder(pretrained=True, out_dim={"rgb": 128, "depth": 32})
        
        # Save the entire model's state_dict
        torch.save(vfe.state_dict(), SAVE_PATH)
        logging.info(f"✅ Successfully saved canonical VFE weights to: {SAVE_PATH}")

if __name__ == "__main__":
    main()