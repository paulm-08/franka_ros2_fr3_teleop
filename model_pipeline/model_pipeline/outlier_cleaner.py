#!/usr/bin/env python3
import numpy as np
import pickle
import argparse
import logging
from pathlib import Path

# --- Logger Setup ---
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()]
)

def find_and_clean_outliers(input_path, output_path, threshold_action, threshold_tactile):
    """
    Scans a trajectory dataset for outliers and interactively removes them.
    """
    logging.info(f"Loading dataset from {input_path}...")
    try:
        with open(input_path, "rb") as f:
            all_trajectories = pickle.load(f)
    except FileNotFoundError:
        logging.error(f"❌ Input file not found: {input_path}")
        return

    logging.info(f"Scanning {len(all_trajectories)} trajectories for outliers...")
    
    corrupted_indices = set()
    outlier_details = []

    for i, traj in enumerate(all_trajectories):
        # Check for outliers in joint actions (delta_q)
        max_action = np.max(np.abs(traj['delta_q']))
        if max_action > threshold_action:
            corrupted_indices.add(i)
            outlier_details.append(
                f"  - Trajectory {i}: Found extreme action value of {max_action:.4f} (Threshold: {threshold_action})"
            )
        
        # Check for outliers in tactile features
        max_tactile = np.max(traj['tactile_t'])
        if max_tactile > threshold_tactile:
            corrupted_indices.add(i)
            outlier_details.append(
                f"  - Trajectory {i}: Found extreme tactile value of {max_tactile:.2f} (Threshold: {threshold_tactile})"
            )
            
    # --- Report and Ask for Confirmation ---
    if not corrupted_indices:
        logging.info("✅ No outliers found. Dataset is clean.")
        return

    logging.warning("\n" + "="*50)
    logging.warning("          🚨 OUTLIERS DETECTED 🚨")
    logging.warning("="*50)
    logging.warning(f"Found {len(corrupted_indices)} trajectories with values exceeding thresholds:")
    for detail in outlier_details:
        logging.warning(detail)
    logging.warning("="*50)
    
    try:
        confirm = input("Do you want to delete these trajectories and save a new, cleaned dataset? (y/n): ")
    except KeyboardInterrupt:
        logging.info("\nOperation cancelled by user.")
        return
        
    if confirm.lower() != 'y':
        logging.info("Operation cancelled. No changes have been made.")
        return
        
    # --- Create and Save the Cleaned Dataset ---
    clean_trajectories = [
        traj for i, traj in enumerate(all_trajectories) if i not in corrupted_indices
    ]
    
    logging.info(f"Removed {len(corrupted_indices)} trajectories. New dataset will have {len(clean_trajectories)} trajectories.")
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        pickle.dump(clean_trajectories, f)
        
    logging.info(f"✅ Successfully saved cleaned dataset to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Clean outlier trajectories from a .pkl dataset.")
    parser.add_argument(
        "--input_file", type=str, required=True,
        help="Path to the input .pkl dataset file to be cleaned."
    )
    parser.add_argument(
        "--output_file", type=str, required=True,
        help="Path to save the new, cleaned .pkl dataset file."
    )
    parser.add_argument(
        "--threshold_action", type=float, default=1.0,
        help="Maximum absolute value allowed for any joint action (delta_q)."
    )
    parser.add_argument(
        "--threshold_tactile", type=float, default=1000.0,
        help="Maximum value allowed for any tactile feature."
    )
    args = parser.parse_args()

    find_and_clean_outliers(
        input_path=Path(args.input_file),
        output_path=Path(args.output_file),
        threshold_action=args.threshold_action,
        threshold_tactile=args.threshold_tactile
    )

if __name__ == "__main__":
    main()
