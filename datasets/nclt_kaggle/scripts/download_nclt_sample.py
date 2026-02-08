#!/usr/bin/env python3
"""Download NCLT dataset sample for local development.

Supports three modes:
  1. Kaggle API download (requires kaggle CLI + API key)
  2. Original NCLT ground truth download (lightweight)
  3. Manual download instructions

Usage:
    python scripts/download_nclt_sample.py --source kaggle
    python scripts/download_nclt_sample.py --source nclt-gt
    python scripts/download_nclt_sample.py --source manual
"""

from __future__ import annotations

import argparse
import hashlib
import logging
import os
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "data"
NCLT_DIR = DATA_DIR / "NCLT_preprocessed"

KAGGLE_DATASET = "creatorofuniverses/nclt-iprofi-hack-23"

# ground truth files from original NCLT (HTTPS, skip SSL verification
# since the NCLT server has certificate issues)
NCLT_GT_BASE = "https://robots.engin.umich.edu/nclt"
NCLT_GT_URLS = {
    "2012-01-08": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-01-08.csv",
    "2012-01-22": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-01-22.csv",
    "2012-02-12": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-02-12.csv",
    "2012-02-18": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-02-18.csv",
    "2012-03-31": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-03-31.csv",
    "2012-05-26": f"{NCLT_GT_BASE}/ground_truth/groundtruth_2012-05-26.csv",
}


def download_file(url: str, dest: Path) -> bool:
    """download a file from URL to dest, return True on success"""
    dest.parent.mkdir(parents=True, exist_ok=True)

    if dest.exists():
        logger.info("File already exists: %s", dest)
        return True

    logger.info("Downloading %s -> %s", url, dest)
    try:
        import ssl

        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, context=ctx) as response, open(dest, "wb") as out:
            out.write(response.read())
        logger.info("Downloaded: %s (%.1f KB)", dest.name, dest.stat().st_size / 1024)
        return True
    except Exception as e:
        logger.error("Failed to download %s: %s", url, e)
        return False


def download_kaggle(output_dir: Path) -> bool:
    """download dataset from Kaggle API, return True on success"""
    try:
        import kaggle  # noqa: F401
    except ImportError:
        logger.error(
            "kaggle package not installed. Run: pip install kaggle\n"
            "Then place your API key in ~/.kaggle/kaggle.json\n"
            "Get your key from: https://www.kaggle.com/settings"
        )
        return False

    kaggle_json = Path.home() / ".kaggle" / "kaggle.json"
    if not kaggle_json.exists():
        logger.error(
            "Kaggle API key not found at %s\n"
            "Download from: https://www.kaggle.com/settings\n"
            "Place it at: %s\n"
            "Then run: chmod 600 %s",
            kaggle_json, kaggle_json, kaggle_json,
        )
        return False

    logger.info("Downloading dataset from Kaggle: %s", KAGGLE_DATASET)
    logger.info("This may take a while depending on your connection...")

    try:
        from kaggle.api.kaggle_api_extended import KaggleApi

        api = KaggleApi()
        api.authenticate()
        api.dataset_download_files(
            KAGGLE_DATASET,
            path=str(output_dir),
            unzip=True,
        )
        logger.info("Kaggle dataset downloaded to %s", output_dir)
        return True
    except Exception as e:
        logger.error("Kaggle download failed: %s", e)
        return False


def download_nclt_groundtruth(output_dir: Path) -> bool:
    """download ground truth CSVs from NCLT website, return True on success"""
    logger.info("Downloading NCLT ground truth files...")
    success = True

    for session, url in NCLT_GT_URLS.items():
        session_dir = output_dir / session
        session_dir.mkdir(parents=True, exist_ok=True)
        dest = session_dir / "track.csv"
        if not download_file(url, dest):
            success = False

    return success


def show_manual_instructions() -> None:
    print(
        "\n"
        "=" * 60 + "\n"
        "  MANUAL DOWNLOAD INSTRUCTIONS\n"
        "=" * 60 + "\n"
        "\n"
        "Option A: Kaggle (Full preprocessed dataset)\n"
        "  1. Go to: https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23\n"
        "  2. Click 'Download' (you need a Kaggle account)\n"
        "  3. Extract to: data/NCLT_preprocessed/\n"
        "\n"
        "Option B: Kaggle API\n"
        "  1. pip install kaggle\n"
        "  2. Go to https://www.kaggle.com/settings -> Create API Token\n"
        "  3. Place kaggle.json in ~/.kaggle/\n"
        "  4. Run: python scripts/download_nclt_sample.py --source kaggle\n"
        "\n"
        "Option C: Original NCLT (ground truth only)\n"
        "  Run: python scripts/download_nclt_sample.py --source nclt-gt\n"
        "\n"
        "Expected structure after download:\n"
        "  data/NCLT_preprocessed/\n"
        "  ├── train.csv\n"
        "  ├── val.csv\n"
        "  ├── test.csv\n"
        "  ├── 2012-01-08/\n"
        "  │   ├── velodyne_data/\n"
        "  │   ├── images_small/\n"
        "  │   └── track.csv\n"
        "  └── ...\n"
        "\n"
        "=" * 60
    )


def verify_dataset(data_dir: Path) -> bool:
    """check dataset directory structure, return True if valid"""
    if not data_dir.exists():
        logger.warning("Dataset directory not found: %s", data_dir)
        return False

    issues = []

    # check for split CSVs
    for csv_name in ["train.csv", "val.csv", "test.csv"]:
        if not (data_dir / csv_name).exists():
            issues.append(f"Missing {csv_name}")

    # check for session directories (directly under data_dir)
    known_sessions = [
        "2012-01-08", "2012-01-22", "2012-02-12", "2012-02-18",
        "2012-03-31", "2012-05-26", "2012-08-04", "2012-10-28",
        "2012-11-04", "2012-12-01",
    ]
    sessions = sorted(
        p.name for p in data_dir.iterdir()
        if p.is_dir() and p.name in known_sessions
    )
    if not sessions:
        issues.append("No session directories found")
    else:
        logger.info("Found %d sessions: %s", len(sessions), ", ".join(sessions))
        for session in sessions:
            session_dir = data_dir / session
            if not (session_dir / "track.csv").exists():
                issues.append(f"Missing track.csv in {session}")

    if issues:
        logger.warning("Dataset verification issues:")
        for issue in issues:
            logger.warning("  - %s", issue)
        return False

    logger.info("Dataset verification passed!")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Download NCLT dataset sample")
    parser.add_argument(
        "--source",
        choices=["kaggle", "nclt-gt", "manual"],
        default="manual",
        help="Download source (default: manual instructions)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=NCLT_DIR,
        help="Output directory for dataset",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Only verify existing dataset",
    )
    args = parser.parse_args()

    if args.verify_only:
        success = verify_dataset(args.output_dir)
        sys.exit(0 if success else 1)

    if args.source == "kaggle":
        success = download_kaggle(args.output_dir)
    elif args.source == "nclt-gt":
        success = download_nclt_groundtruth(args.output_dir)
    else:
        show_manual_instructions()
        success = True

    if success:
        verify_dataset(args.output_dir)


if __name__ == "__main__":
    main()
