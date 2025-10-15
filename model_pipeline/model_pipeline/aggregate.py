# aggregate.py
import shutil
from pathlib import Path
import argparse
import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

def aggregate_folders(source_dirs, dest_dir):
    """
    Copies all files from a list of source directories into a single destination directory.
    """
    dest_path = Path(dest_dir)
    dest_path.mkdir(parents=True, exist_ok=True)
    
    total_files = 0
    for source_dir in source_dirs:
        source_path = Path(source_dir)
        if not source_path.is_dir():
            logging.warning(f"Source '{source_dir}' is not a valid directory. Skipping.")
            continue
            
        logging.info(f"Copying files from '{source_path}' to '{dest_path}'...")
        for file_path in source_path.glob('*'):
            if file_path.is_file():
                shutil.copy(file_path, dest_path / file_path.name)
                total_files += 1
    
    logging.info(f"✅ Aggregated {total_files} files into '{dest_path}'.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Aggregate files from multiple folders into one.")
    parser.add_argument("--sources", nargs="+", required=True, help="List of source directories to pull files from.")
    parser.add_argument("--dest", type=str, required=True, help="Single destination directory.")
    args = parser.parse_args()
    
    aggregate_folders(args.sources, args.dest)