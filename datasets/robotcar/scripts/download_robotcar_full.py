#!/usr/bin/env python3
"""Download Oxford RobotCar full dataset for ORB-SLAM3 experiments

Downloads GPS/INS and stereo camera data for specific sessions from the
Oxford RobotCar Dataset (robotcar-dataset.robots.ox.ac.uk)

Requires a registered account at mrgdatashare.robots.ox.ac.uk

Usage:
    # phase 1: GPS/INS only (tiny, lets you do Mono-Inertial with existing Seasons images)
    python download_robotcar_full.py --username USER --password PASS --phase 1

    # phase 2: Add stereo images (large, allows Stereo-Inertial)
    python download_robotcar_full.py --username USER --password PASS --phase 2

    # download specific sensors for specific sessions
    python download_robotcar_full.py --username USER --password PASS \
        --sessions 2014-11-28-12-07-13 --sensors gps,vo
"""

import argparse
import os
import sys
import tarfile
import time
from pathlib import Path

import requests
from lxml import html

LOGIN_URL = "https://mrgdatashare.robots.ox.ac.uk/"
DOWNLOAD_BASE = "http://mrgdatashare.robots.ox.ac.uk:80/download/?filename=datasets/"

# RobotCar Seasons session mapping (condition -> full dataset timestamp)
SEASONS_SESSIONS = {
    "overcast-reference": "2014-11-28-12-07-13",
    "dawn": "2014-12-16-18-44-24",
    "dusk": "2015-02-20-16-34-06",
    "night": "2014-12-10-18-10-50",
    "night-rain": "2014-12-17-18-18-43",
    "overcast-summer": "2015-05-22-11-14-30",
    "overcast-winter": "2015-11-13-10-28-08",
    "rain": "2014-11-25-09-18-32",
    "snow": "2015-02-03-08-45-10",
    "sun": "2015-03-10-14-18-10",
}

# priority sessions for ORB-SLAM3
PRIORITY_SESSIONS = [
    "2014-11-28-12-07-13",  # overcast-reference (reference session)
    "2015-02-03-08-45-10",  # snow (seasonal change test)
]

# sensor chunks per session (from datasets.csv)
SESSION_CHUNKS = {
    "2014-11-28-12-07-13": {
        "gps": ["gps"],
        "vo": ["vo"],
        "stereo_left": [f"stereo_left_{i:02d}" for i in range(1, 7)],
        "stereo_right": [f"stereo_right_{i:02d}" for i in range(1, 7)],
        "stereo_centre": [f"stereo_centre_{i:02d}" for i in range(1, 7)],
        "mono_rear": [f"mono_rear_{i:02d}" for i in range(1, 7)],
        "lms_front": [f"lms_front_{i:02d}" for i in range(1, 7)],
    },
    "2015-02-03-08-45-10": {
        "gps": ["gps"],
        "vo": ["vo"],
        "stereo_left": [f"stereo_left_{i:02d}" for i in range(1, 8)],
        "stereo_right": [f"stereo_right_{i:02d}" for i in range(1, 8)],
        "stereo_centre": [f"stereo_centre_{i:02d}" for i in range(1, 8)],
        "mono_rear": [f"mono_rear_{i:02d}" for i in range(1, 8)],
        "lms_front": [f"lms_front_{i:02d}" for i in range(1, 8)],
    },
}

# download phases
PHASE_SENSORS = {
    1: ["gps", "vo"],                         # ~100 MB total
    2: ["stereo_left", "stereo_right"],        # ~120 GB total
    3: ["lms_front"],                          # ~2.4 GB total
}

OUTPUT_DIR = Path("/workspace/data/robotcar_full")


# authentication
def login(username: str, password: str) -> requests.Session:
    """Authenticate with mrgdatashare and return a session with cookies"""
    session = requests.Session()

    print("Connecting to mrgdatashare.robots.ox.ac.uk...")
    response = session.get(LOGIN_URL)
    if response.status_code != 200:
        raise ConnectionError(f"Failed to reach login page: HTTP {response.status_code}")

    tree = html.fromstring(response.text)
    tokens = tree.xpath("//input[@name='csrfmiddlewaretoken']/@value")
    if not tokens:
        raise ValueError("Could not find CSRF token on login page")

    csrf_token = tokens[0]

    payload = {
        "username": username,
        "password": password,
        "csrfmiddlewaretoken": csrf_token,
    }
    result = session.post(LOGIN_URL, data=payload, headers={"Referer": LOGIN_URL})

    if result.status_code != 200 or "Please try again" in result.text:
        raise ValueError(
            "Login failed. Check your username and password.\n"
            "Register at: https://mrgdatashare.robots.ox.ac.uk/register/"
        )

    print("Logged in successfully!")
    return session


def download_file(session: requests.Session, session_id: str, chunk_name: str,
                  output_dir: Path, extract: bool = True) -> bool:
    """Download and optionally extract a single .tar file"""
    url = f"{DOWNLOAD_BASE}{session_id}/{session_id}_{chunk_name}.tar"
    tar_path = output_dir / f"{session_id}_{chunk_name}.tar"

    if tar_path.exists():
        print(f"  Already downloaded: {tar_path.name}")
    else:
        print(f"  Downloading: {chunk_name}.tar ...", end=" ", flush=True)
        try:
            response = session.get(url, stream=True, timeout=600)
        except requests.exceptions.RequestException as e:
            print(f"FAILED ({e})")
            return False

        if response.status_code != 200:
            print(f"FAILED (HTTP {response.status_code})")
            return False

        content_type = response.headers.get("content-type", "")
        if "html" in content_type:
            print("FAILED (session expired, got HTML)")
            return False

        total_size = int(response.headers.get("content-length", 0))
        downloaded = 0

        with open(tar_path, "wb") as f:
            for chunk in response.iter_content(chunk_size=1024 * 1024):
                if b"File not found" in chunk:
                    print("FAILED (file not found on server)")
                    tar_path.unlink(missing_ok=True)
                    return False
                f.write(chunk)
                downloaded += len(chunk)
                if total_size > 0:
                    pct = downloaded / total_size * 100
                    mb = downloaded / 1024 / 1024
                    total_mb = total_size / 1024 / 1024
                    print(
                        f"\r  Downloading: {chunk_name}.tar ... "
                        f"{mb:.0f}/{total_mb:.0f} MB ({pct:.1f}%)",
                        end="", flush=True,
                    )

        print(f" OK ({downloaded / 1024 / 1024:.1f} MB)")

    if extract and tar_path.exists():
        print(f"  Extracting: {tar_path.name} ...", end=" ", flush=True)
        try:
            with tarfile.open(tar_path, "r:*") as tar:
                tar.extractall(path=output_dir)
            print("OK")
            tar_path.unlink()
            print(f"  Removed tar: {tar_path.name}")
        except (tarfile.ReadError, EOFError) as e:
            print(f"FAILED ({e})")
            return False

    return True


def download_sensor(session: requests.Session, session_id: str, sensor: str,
                    output_dir: Path) -> bool:
    """Download all chunks for a sensor"""
    chunks = SESSION_CHUNKS.get(session_id, {}).get(sensor, [])
    if not chunks:
        print(f"  Warning: no chunks defined for {session_id}/{sensor}")
        chunks = [sensor]

    success = True
    for chunk in chunks:
        if not download_file(session, session_id, chunk, output_dir):
            success = False
    return success


def check_existing_data(session_id: str, output_dir: Path) -> dict:
    """Check what data already exists for a session"""
    session_dir = output_dir / session_id
    existing = {}

    if not session_dir.exists():
        return existing

    gps_dir = session_dir / "gps"
    ins_file = gps_dir / "ins.csv"
    if ins_file.exists():
        existing["gps/ins.csv"] = True

    for view in ["left", "centre", "right"]:
        stereo_dir = session_dir / "stereo" / view
        if stereo_dir.exists():
            n_images = len(list(stereo_dir.glob("*.png")))
            if n_images > 0:
                existing[f"stereo/{view}"] = f"{n_images} images"

    vo_dir = session_dir / "vo"
    if vo_dir.exists() and list(vo_dir.glob("*.csv")):
        existing["vo"] = True

    return existing


def print_status(sessions: list, output_dir: Path):
    """Print current download status"""
    print("\nCurrent data status:")
    for sid in sessions:
        existing = check_existing_data(sid, output_dir)
        condition = next((k for k, v in SEASONS_SESSIONS.items() if v == sid), "?")
        print(f"\n  {sid} ({condition}):")
        if not existing:
            print("    No data downloaded yet")
        else:
            for key, val in existing.items():
                print(f"    {key}: {val}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Download Oxford RobotCar data for ORB-SLAM3 experiments"
    )
    parser.add_argument("--username", required=True,
                        help="mrgdatashare.robots.ox.ac.uk username")
    parser.add_argument("--password", required=True,
                        help="mrgdatashare.robots.ox.ac.uk password")
    parser.add_argument("--phase", type=int, choices=[1, 2, 3],
                        help="Download phase: 1=GPS/INS+VO (~100MB), "
                             "2=stereo (~120GB), 3=LiDAR (~2.4GB)")
    parser.add_argument("--sessions", type=str, default=None,
                        help="Comma-separated session IDs (default: priority sessions)")
    parser.add_argument("--sensors", type=str, default=None,
                        help="Comma-separated sensor names (overrides --phase)")
    parser.add_argument("--output-dir", type=str, default=str(OUTPUT_DIR),
                        help=f"Output directory (default: {OUTPUT_DIR})")
    parser.add_argument("--no-extract", action="store_true",
                        help="Keep .tar files without extracting")
    parser.add_argument("--status", action="store_true",
                        help="Just show current download status")

    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.sessions:
        sessions = args.sessions.split(",")
    else:
        sessions = PRIORITY_SESSIONS

    if args.status:
        print_status(sessions, output_dir)
        return

    if args.sensors:
        sensors = args.sensors.split(",")
    elif args.phase:
        sensors = PHASE_SENSORS[args.phase]
    else:
        parser.error("Specify --phase or --sensors")
        return

    print("\nOxford RobotCar Dataset Downloader")
    print(f"Sessions: {sessions}")
    print(f"Sensors:  {sensors}")
    print(f"Output:   {output_dir}")

    import shutil
    free_gb = shutil.disk_usage(output_dir).free / (1024**3)
    print(f"Available disk space: {free_gb:.1f} GB")
    print()

    session = login(args.username, args.password)

    for sid in sessions:
        condition = next((k for k, v in SEASONS_SESSIONS.items() if v == sid), "unknown")
        print(f"\n{'='*60}")
        print(f"Session: {sid} ({condition})")
        print(f"{'='*60}")

        for sensor in sensors:
            print(f"\n  Sensor: {sensor}")
            download_sensor(session, sid, sensor, output_dir)

        time.sleep(1)

    print_status(sessions, output_dir)
    print("Download complete!")


if __name__ == "__main__":
    main()
