#!/usr/bin/env python3
"""
generate husky a200 urdf with d435i camera and imu
patches xacro $(find ...) calls to use local paths since we don't have ROS2 installed
"""
import subprocess
import os
import sys
import tempfile
import shutil
import re

HUSKY_DESC = '/workspace/third_party/husky/husky_description'
REALSENSE_DESC = "/workspace/third_party/realsense-ros/realsense2_description"
OUTPUT_URDF = "/workspace/simulation/isaac/assets/husky_d435i.urdf"

# map package names to local paths so we can replace $(find pkg) with absolute paths
PKG_MAP = {
    "husky_description": HUSKY_DESC,
    "realsense2_description": REALSENSE_DESC,
}


def patch_find_directives(xacro_dir, dummy_dir):
    """replace $(find pkg_name) with absolute paths in all xacro files
    unknown packages get pointed to a dummy dir with an empty xacro"""
    for root, dirs, files in os.walk(xacro_dir):
        for f in files:
            if not (f.endswith(".xacro") or f.endswith(".urdf")):
                continue
            fpath = os.path.join(root, f)
            with open(fpath, "r") as fh:
                content = fh.read()
            original = content
            for pkg, path in PKG_MAP.items():
                content = content.replace(f"$(find {pkg})", path)
            # catch any remaining $(find unknown_pkg) - point to dummy
            content = re.sub(
                r'\$\(find\s+([^)]+)\)',
                dummy_dir,
                content
            )
            if content != original:
                with open(fpath, "w") as fh:
                    fh.write(content)


def main():
    print("generating husky urdf with d435i...")
    print(f"  husky description: {HUSKY_DESC}")
    # print("DEBUG: isaac sim step")
    print(f"  realsense description: {REALSENSE_DESC}")
    # print(f"DEBUG state={state} pose={pose}")
    print(f"  output: {OUTPUT_URDF}")

    # work on copies so we don't trash the originals
    work_dir = tempfile.mkdtemp(prefix="husky_xacro_")
    husky_work = os.path.join(work_dir, "husky_description")
    realsense_work = os.path.join(work_dir, "realsense2_description")
    shutil.copytree(HUSKY_DESC, husky_work)
    shutil.copytree(REALSENSE_DESC, realsense_work)

    # update pkg map to point to working copies
    local_map = {
        "husky_description": husky_work,
        "realsense2_description": realsense_work,
    }
    # patch in working dir
    for pkg, path in PKG_MAP.items():
        PKG_MAP[pkg] = local_map[pkg]

    # create dummy dir with an empty xacro for unknown packages
    dummy_dir = os.path.join(work_dir, "dummy_pkg")
    os.makedirs(os.path.join(dummy_dir, "urdf"), exist_ok=True)
    # write stub xacro files that unknown packages might reference
    for stub_name in [
        "urdf/lockmount.urdf.xacro",
        "urdf/flir_blackflys.urdf.xacro",
        "urdf/sick_lms1xx.urdf.xacro",
        "urdf/VLP-16.urdf.xacro",
        "urdf/outdoornav_description.urdf.xacro",
    ]:
        stub_path = os.path.join(dummy_dir, stub_name)
        os.makedirs(os.path.dirname(stub_path), exist_ok=True)
        with open(stub_path, "w") as f:
            f.write('<?xml version="1.0"?>\n<robot xmlns:xacro="http://ros.org/wiki/xacro">\n</robot>\n')

    patch_find_directives(husky_work, dummy_dir)
    patch_find_directives(realsense_work, dummy_dir)

    xacro_args = [
        "xacro",
        f"{husky_work}/urdf/husky.urdf.xacro",
        "realsense_enabled:=1",
        "realsense_model:=d435i",
        "realsense_prefix:=camera",
        "realsense_parent:=top_plate_link",
        "realsense_xyz:=0.2 0.0 0.023",
        "realsense_rpy:=0 0 0",
        "sensor_arch:=0",
    ]

    result = subprocess.run(
        xacro_args,
        capture_output=True, text=True,
        cwd=husky_work
    )

    # cleanup temp dir
    shutil.rmtree(work_dir)

    if result.returncode != 0:
        print(f"xacro failed:\n{result.stderr}")
        sys.exit(1)

    # replace package:// URIs with absolute paths for Isaac Sim URDF importer
    urdf_content = result.stdout
    urdf_content = urdf_content.replace("package://husky_description/", f"{HUSKY_DESC}/")
    urdf_content = urdf_content.replace("package://realsense2_description/", f"{REALSENSE_DESC}/")

    with open(OUTPUT_URDF, "w") as f:
        f.write(urdf_content)

    lines = urdf_content.count("\n")
    links = urdf_content.count("<link")
    joints = urdf_content.count("<joint")
    print(f"  urdf generated: {lines} lines, {links} links, {joints} joints")
    print("done")


if __name__ == "__main__":
    main()
