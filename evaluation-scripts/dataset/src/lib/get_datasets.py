import os
from pathlib import Path

BAG_REPO = os.environ.get("BAG_REPO", "/data")

def get_sample_list() -> list[str]:
    root_folder = Path(BAG_REPO)
    prohibited_folders = ["out", "gt", "trackers", "bag", "tmp", "compressed", "original"]
    return [folder.name for folder in root_folder.iterdir() if (folder.is_dir() and folder.name not in prohibited_folders)]
