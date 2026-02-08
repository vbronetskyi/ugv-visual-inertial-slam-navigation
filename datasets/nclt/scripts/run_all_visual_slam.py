#!/usr/bin/env python3
"""Overnight deep visual SLAM benchmark on NCLT.

Runs DROID-SLAM (Teed & Deng, NeurIPS 2021), DPVO (Teed et al., 2024) and
DPV-SLAM (Lipson et al., ECCV 2024) on NCLT spring (2012-04-29) and summer
(2012-08-04). all three take the same monocular Cam0 stream at 808x616
half-res, calibrated via COLMAP (f=221, cx=404, cy=308).

heads up - exp 0.5 later found Cam0 is the top/sky-facing camera. kept here
anyway so results stay comparable with exp 0.2-0.4; the side-cam rerun is a
separate script. best to launch inside tmux because each session is ~1-2h.
"""

import os
import sys
import subprocess
import time
import json
import shutil
import traceback
import signal
import multiprocessing as mp
from pathlib import Path
from datetime import datetime, timedelta

import numpy as np
import cv2

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from scipy.spatial.transform import Rotation


BASE_DIR = Path("/workspace/datasets/nclt")
DATA_DIR = Path("/workspace/nclt_data")
RESULTS_DIR = BASE_DIR / "results" / "week0_visual_slam"
INSTALL_DIR = Path("/tmp/visual_slam_methods")
VENV_DIR = Path("/tmp/vslam_venv")
PREPARED_BASE = DATA_DIR / "images_prepared"

SESSIONS = ["2012-04-29", "2012-08-04"]
SESSION_NAMES = {"2012-04-29": "spring", "2012-08-04": "summer"}

# camera intrinsics (COLMAP-calibrated on 300 frames of Cam5, half-res 808x616).
# The original NCLT cam_params.zip is 404 from the server; these f/cx/cy are the
# best pinhole approximation we have. For a fisheye equidistant model the
# "true" focal would be ~579, not 221; see CHANGELOG for why this matters.
FX, FY = 221.0, 221.0
CX, CY = 404.0, 308.0
IMG_W, IMG_H = 808, 616

# reference LiDAR ICP results from Experiment 0.1 for comparison
LIDAR_ATE = {"2012-04-29": 174.0, "2012-08-04": 188.2}

METHODS = {
    "droid_slam": {
        "name": "DROID-SLAM",
        "repo": "https://github.com/princeton-vl/DROID-SLAM",
        "timeout_h": 4,
    },
    "dpvo": {
        "name": "DPVO",
        "repo": "https://github.com/princeton-vl/DPVO",
        "timeout_h": 2,
    },
    "dpv_slam": {
        "name": "DPV-SLAM",
        "repo": None,  # uses DPVO repo
        "timeout_h": 3,
    },
}

START_TIME = None
LOG_FILE = None


# embedded runner scripts

DROID_RUNNER_SCRIPT = r'''#!/usr/bin/env python3
"""DROID-SLAM runner called via subprocess from the main script"""
import sys, os, json, time, traceback

def main():
    config = json.loads(sys.argv[1])
    imagedir     = config["imagedir"]
    calib_file   = config["calib"]
    output_traj  = config["output_trajectory"]
    output_stats = config["output_stats"]
    weights_path = config["weights"]
    droid_dir    = config["droid_dir"]
    stride       = config.get("stride", 1)
    buffer_size  = config.get("buffer", 512)

    sys.path.insert(0, droid_dir)
    sys.path.insert(0, os.path.join(droid_dir, "droid_slam"))

    import torch
    import numpy as np
    import cv2
    try:
        from tqdm import tqdm
    except ImportError:
        tqdm = lambda x, **kw: x

    print(f"PyTorch {torch.__version__}, CUDA {torch.cuda.is_available()}", flush=True)
    if torch.cuda.is_available():
        print(f"GPU: {torch.cuda.get_device_name(0)} "
              f"({torch.cuda.get_device_properties(0).total_memory/1e9:.1f} GB)", flush=True)

    from droid import Droid
    from types import SimpleNamespace

    # --- calibration ---
    calib = np.loadtxt(calib_file)
    fx, fy, cx, cy = calib[:4]
    distort = calib[4:] if len(calib) > 4 else None
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

    # --- image list ---
    exts = {".png", ".jpg", ".jpeg"}
    images = sorted(f for f in os.listdir(imagedir)
                    if os.path.splitext(f)[1].lower() in exts)
    images = images[::stride]
    print(f"Images: {len(images)} (stride={stride})", flush=True)
    if not images:
        print("ERROR: no images", flush=True); sys.exit(1)

    # --- compute resize dims (match DROID-SLAM convention) ---
    img0 = cv2.imread(os.path.join(imagedir, images[0]))
    h0, w0 = img0.shape[:2]
    sc = np.sqrt((384 * 512) / (h0 * w0))
    h1 = int(h0 * sc) // 8 * 8
    w1 = int(w0 * sc) // 8 * 8
    sx, sy = w1 / w0, h1 / h0
    print(f"Resize: {w0}x{h0} -> {w1}x{h1}", flush=True)

    # --- args ---
    args = SimpleNamespace(
        weights=weights_path,
        image_size=[h1, w1],
        buffer=buffer_size,
        stereo=False,
        disable_vis=True,
        filter_thresh=1.75,
        beta=0.3,
        warmup=12,
        keyframe_thresh=3.5,
        frontend_thresh=16.0,
        frontend_window=25,
        frontend_radius=2,
        frontend_nms=1,
        backend_thresh=22.0,
        backend_radius=2,
        backend_nms=3,
        upsample=False,
        frontend_device="cuda:0",
        backend_device="cuda:0",
    )

    droid = Droid(args)

    timestamps = []
    t0 = time.time()
    gpu_peak = 0

    # --- tracking ---
    print("=== Tracking ===", flush=True)
    for idx, name in enumerate(tqdm(images, file=sys.stdout)):
        try:
            ts_us = int(os.path.splitext(name)[0])
            ts_s  = ts_us / 1e6
            timestamps.append(ts_s)

            img = cv2.imread(os.path.join(imagedir, name))
            if img is None:
                continue
            if distort is not None and len(distort) >= 4:
                img = cv2.undistort(img, K, distort)
            img = cv2.resize(img, (w1, h1))
            img = torch.as_tensor(img).permute(2, 0, 1)[None]  # [1,C,H,W] uint8
            intrinsics = torch.as_tensor([fx*sx, fy*sy, cx*sx, cy*sy])

            droid.track(ts_s, img, intrinsics=intrinsics)

            mem = torch.cuda.max_memory_allocated() / 1e9
            if mem > gpu_peak:
                gpu_peak = mem

            if (idx + 1) % 2000 == 0:
                print(f"  [{idx+1}/{len(images)}] GPU={mem:.1f}GB", flush=True)
        except RuntimeError as e:
            if "out of memory" in str(e).lower():
                print(f"OOM at frame {idx}, flushing cache", flush=True)
                torch.cuda.empty_cache()
            else:
                print(f"Error frame {idx}: {e}", flush=True)
        except Exception as e:
            print(f"Error frame {idx}: {e}", flush=True)

    track_time = time.time() - t0
    print(f"Tracking: {track_time:.0f}s ({len(images)/track_time:.1f} fps)", flush=True)

    # --- global optimisation + trajectory filling ---
    print("=== Global BA + trajectory fill ===", flush=True)
    t_opt = time.time()
    try:
        def image_replay():
            for name in images:
                ts_us = int(os.path.splitext(name)[0])
                ts_s  = ts_us / 1e6
                img = cv2.imread(os.path.join(imagedir, name))
                if distort is not None and len(distort) >= 4:
                    img = cv2.undistort(img, K, distort)
                img = cv2.resize(img, (w1, h1))
                img = torch.as_tensor(img).permute(2, 0, 1)[None]
                intrinsics = torch.as_tensor([fx*sx, fy*sy, cx*sx, cy*sy])
                yield ts_s, img, intrinsics

        traj = droid.terminate(stream=image_replay())
    except Exception as e:
        print(f"Trajectory fill failed ({e}), extracting keyframe poses", flush=True)
        torch.cuda.empty_cache()
        try:
            # extract keyframe-only poses directly from the video buffer
            n_kf = droid.video.counter.value
            kf_poses = droid.video.poses[:n_kf].cpu().numpy()  # [N,7] SE3
            kf_tstamps = droid.video.tstamp[:n_kf].cpu().numpy()

            # invert poses (camera-to-world) like terminate() does
            from lietorch import SE3
            poses_se3 = SE3(droid.video.poses[:n_kf])
            traj = poses_se3.inv().data.cpu().numpy()

            # replace timestamps with actual ones
            timestamps = kf_tstamps.tolist()
            print(f"Extracted {n_kf} keyframe poses (no trajectory fill)", flush=True)
        except Exception as e2:
            print(f"Keyframe extraction also failed: {e2}", flush=True)
            traj = np.zeros((0, 7))

    opt_time = time.time() - t_opt
    total_time = time.time() - t0
    print(f"Optimisation: {opt_time:.0f}s, total: {total_time:.0f}s", flush=True)
    print(f"Trajectory: {len(traj)} poses", flush=True)

    # --- save TUM trajectory ---
    with open(output_traj, "w") as f:
        for i, pose in enumerate(traj):
            ts = timestamps[i] if i < len(timestamps) else timestamps[-1]
            f.write(f"{ts:.6f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
                    f"{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f}\n")

    # --- save stats ---
    stats = dict(
        method="DROID-SLAM", runtime_total_s=total_time,
        runtime_tracking_s=track_time, runtime_optim_s=opt_time,
        n_images=len(images), n_poses=len(traj),
        tracking_rate=len(traj)/len(images) if images else 0,
        fps=len(images)/track_time if track_time > 0 else 0,
        gpu_memory_peak_gb=gpu_peak, stride=stride, buffer=buffer_size,
    )
    with open(output_stats, "w") as f:
        json.dump(stats, f, indent=2)

    print(f"DONE: {len(traj)} poses, {total_time:.0f}s, GPU peak {gpu_peak:.1f}GB",
          flush=True)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"FATAL: {e}", flush=True)
        traceback.print_exc()
        sys.exit(1)
'''

DPVO_RUNNER_SCRIPT = r'''#!/usr/bin/env python3
"""DPVO / DPV-SLAM runner called via subprocess from the main script"""
import sys, os, json, time, traceback, glob, subprocess

def main():
    config = json.loads(sys.argv[1])
    imagedir     = config["imagedir"]
    calib_file   = config["calib"]
    output_traj  = config["output_trajectory"]
    output_stats = config["output_stats"]
    weights_path = config["weights"]
    dpvo_dir     = config["dpvo_dir"]
    method       = config.get("method", "dpvo")
    stride       = config.get("stride", 1)

    sys.path.insert(0, dpvo_dir)

    import torch
    import numpy as np
    import cv2
    try:
        from tqdm import tqdm
    except ImportError:
        tqdm = lambda x, **kw: x

    print(f"PyTorch {torch.__version__}, CUDA {torch.cuda.is_available()}", flush=True)
    print(f"Method: {method}", flush=True)

    # --- calibration ---
    calib = np.loadtxt(calib_file)
    fx, fy, cx, cy = calib[:4]

    # --- image list ---
    exts = {".png", ".jpg", ".jpeg"}
    images = sorted(f for f in os.listdir(imagedir)
                    if os.path.splitext(f)[1].lower() in exts)
    images = images[::stride]
    print(f"Images: {len(images)} (stride={stride})", flush=True)
    if not images:
        print("ERROR: no images", flush=True); sys.exit(1)

    img0 = cv2.imread(os.path.join(imagedir, images[0]))
    h0, w0 = img0.shape[:2]
    print(f"Image size: {w0}x{h0}", flush=True)

    # DPVO expects half-resolution images (like its own image_stream does)
    h_half, w_half = h0 // 2, w0 // 2
    # crop to multiple of 16
    h_half = h_half - h_half % 16
    w_half = w_half - w_half % 16
    fx_h, fy_h, cx_h, cy_h = fx / 2, fy / 2, cx / 2, cy / 2
    print(f"DPVO half-res: {w_half}x{h_half}, intrinsics: f={fx_h:.1f}", flush=True)

    # --- Try Python API first ---
    traj_poses = None
    timestamps = []
    t0 = time.time()
    gpu_peak = 0

    try:
        from dpvo.dpvo import DPVO
        from dpvo.config import cfg as base_cfg

        cfg = base_cfg.clone()
        cfg.BUFFER_SIZE = 2048  # default 4096, enough for ~2k keyframes
        cfg.PATCHES_PER_FRAME = 80  # default
        if method == "dpv_slam":
            cfg.LOOP_CLOSURE = True
            print("Loop closure ON (DPV-SLAM)", flush=True)
        else:
            cfg.LOOP_CLOSURE = False
            print("Loop closure OFF (DPVO)", flush=True)

        slam = DPVO(cfg, weights_path, ht=h_half, wd=w_half, viz=None)

        print("=== Processing frames ===", flush=True)
        with torch.no_grad():
          for idx, name in enumerate(tqdm(images, file=sys.stdout)):
            try:
                ts_us = int(os.path.splitext(name)[0])
                ts_s  = ts_us / 1e6
                timestamps.append(ts_s)

                img = cv2.imread(os.path.join(imagedir, name))
                if img is None:
                    continue
                img = cv2.resize(img, (w_half, h_half), interpolation=cv2.INTER_AREA)
                img = torch.from_numpy(img).permute(2, 0, 1).cuda()  # [C,H,W] uint8
                intrinsics = torch.from_numpy(np.array([fx_h, fy_h, cx_h, cy_h], dtype=np.float64)).cuda()

                slam(ts_s, img, intrinsics)

                mem = torch.cuda.max_memory_allocated() / 1e9
                if mem > gpu_peak:
                    gpu_peak = mem

                if (idx + 1) % 2000 == 0:
                    print(f"  [{idx+1}/{len(images)}] GPU={mem:.1f}GB", flush=True)

            except RuntimeError as e:
                if "out of memory" in str(e).lower():
                    print(f"OOM at frame {idx}, flushing", flush=True)
                    torch.cuda.empty_cache()
                else:
                    print(f"Error frame {idx}: {e}", flush=True)
            except Exception as e:
                print(f"Error frame {idx}: {e}", flush=True)

        # terminate: runs final global BA, interpolates missing poses, inverts
        print("Terminating (final BA + interpolation)...", flush=True)
        try:
            torch.cuda.empty_cache()
            poses_np, tstamps_np = slam.terminate()
            traj_poses = poses_np  # [N, 7] = [tx,ty,tz, qx,qy,qz, qw]
            timestamps = tstamps_np.tolist()
            print(f"Extracted {len(traj_poses)} poses (full trajectory)", flush=True)
        except Exception as e_term:
            print(f"terminate() failed ({e_term}), extracting keyframe poses via CPU", flush=True)
            torch.cuda.empty_cache()
            try:
                n_kf = slam.n
                # copy raw poses to CPU numpy; handle torch, lietorch, or numpy
                p = slam.pg.poses_[:n_kf]
                raw_poses = p.detach().cpu().numpy() if hasattr(p, 'detach') else (
                    p.cpu().numpy() if hasattr(p, 'cpu') else np.array(p))
                t = slam.pg.tstamps_[:n_kf]
                kf_tstamps = t.detach().cpu().numpy() if hasattr(t, 'detach') else (
                    t.cpu().numpy() if hasattr(t, 'cpu') else np.array(t))

                # invert SE3 poses in numpy (camera-to-world):
                # pose = [tx,ty,tz, qx,qy,qz,qw], invert quaternion + recompute translation
                from scipy.spatial.transform import Rotation
                inv_poses = []
                for p in raw_poses:
                    t = p[:3]
                    q = p[3:]  # [qx,qy,qz,qw]
                    R = Rotation.from_quat(q).as_matrix()
                    R_inv = R.T
                    t_inv = -R_inv @ t
                    q_inv = Rotation.from_matrix(R_inv).as_quat()  # [qx,qy,qz,qw]
                    inv_poses.append(np.concatenate([t_inv, q_inv]))
                traj_poses = np.array(inv_poses)

                timestamps = [slam.tlist[int(t)] if int(t) < len(slam.tlist)
                              else float(t) for t in kf_tstamps]
                print(f"Extracted {n_kf} keyframe poses via CPU (no trajectory fill)", flush=True)
            except Exception as e2:
                print(f"Keyframe extraction also failed: {e2}", flush=True)
                import traceback; traceback.print_exc()

    except ImportError as e:
        print(f"DPVO Python API unavailable ({e}), trying demo.py", flush=True)

        # --- Fallback: call demo.py ---
        demo_py = os.path.join(dpvo_dir, "demo.py")
        out_name = os.path.splitext(output_traj)[0]
        cmd = [
            sys.executable, demo_py,
            f"--imagedir={imagedir}",
            f"--calib={calib_file}",
            f"--network={weights_path}",
            f"--stride={stride}",
            "--save_trajectory",
            f"--name={out_name}",
        ]
        if method == "dpv_slam":
            cmd += ["--opts", "LOOP_CLOSURE", "True"]

        print(f"CMD: {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=3*3600, cwd=dpvo_dir)
        print(result.stdout[-2000:] if result.stdout else "", flush=True)
        if result.returncode != 0:
            print(f"STDERR: {result.stderr[-2000:]}", flush=True)

        # find trajectory files
        patterns = [f"{out_name}*.tum", f"{out_name}*.txt",
                    "*.tum", "*trajectory*"]
        search_dirs = [os.path.dirname(output_traj), dpvo_dir]
        for d in search_dirs:
            for pat in patterns:
                for fpath in glob.glob(os.path.join(d, pat)):
                    try:
                        data = np.loadtxt(fpath)
                        if data.ndim == 2 and data.shape[1] >= 7:
                            traj_poses = data[:, 1:8] if data.shape[1] == 8 else data[:, :7]
                            timestamps = (data[:, 0] if data.shape[1] == 8
                                          else np.arange(len(data))).tolist()
                            print(f"Found trajectory: {fpath} ({len(data)} poses)", flush=True)
                            break
                    except Exception:
                        pass
                if traj_poses is not None:
                    break
            if traj_poses is not None:
                break

    total_time = time.time() - t0
    n_poses = len(traj_poses) if traj_poses is not None else 0
    print(f"Total time: {total_time:.0f}s, poses: {n_poses}", flush=True)

    # --- save TUM trajectory ---
    if traj_poses is not None and n_poses > 0:
        with open(output_traj, "w") as f:
            for i, pose in enumerate(traj_poses):
                ts = timestamps[i] if i < len(timestamps) else (
                    timestamps[-1] + (i - len(timestamps) + 1) * 0.2 if timestamps
                    else float(i))
                if len(pose) >= 7:
                    f.write(f"{ts:.6f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
                            f"{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f} {pose[6]:.6f}\n")
                elif len(pose) >= 3:
                    f.write(f"{ts:.6f} {pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
                            f"0.0 0.0 0.0 1.0\n")
    else:
        open(output_traj, "w").close()  # empty file

    stats = dict(
        method=method.upper().replace("_", "-"),
        runtime_s=total_time, n_images=len(images), n_poses=n_poses,
        tracking_rate=n_poses/len(images) if images else 0,
        fps=len(images)/total_time if total_time > 0 else 0,
        gpu_memory_peak_gb=gpu_peak, stride=stride,
    )
    with open(output_stats, "w") as f:
        json.dump(stats, f, indent=2)

    print(f"DONE: {n_poses} poses, {total_time:.0f}s, GPU peak {gpu_peak:.1f}GB",
          flush=True)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"FATAL: {e}", flush=True)
        traceback.print_exc()
        sys.exit(1)
'''


# logging & utilities

def log(msg, level="INFO"):
    """Print and log a timestamped message"""
    ts = datetime.now().strftime("%H:%M:%S")
    elapsed = ""
    if START_TIME:
        el = datetime.now() - START_TIME
        elapsed = f" [{str(el).split('.')[0]}]"
    line = f"[{ts}{elapsed}] {level}: {msg}"
    print(line, flush=True)
    if LOG_FILE:
        try:
            with open(LOG_FILE, "a") as f:
                f.write(line + "\n")
        except Exception:
            pass


def run_cmd(cmd, timeout=600, cwd=None, env=None, capture=True):
    """Run a shell command with timeout. Returns CompletedProcess"""
    if isinstance(cmd, str):
        cmd = cmd.split()
    cmd_str = " ".join(str(c) for c in cmd)[:200]
    log(f"  $ {cmd_str}", level="CMD")
    merged_env = dict(os.environ)
    if env:
        merged_env.update(env)
    try:
        r = subprocess.run(
            cmd, capture_output=capture, text=True,
            timeout=timeout, cwd=cwd, env=merged_env,
        )
        if r.returncode != 0 and capture:
            stderr_tail = (r.stderr or "")[-500:]
            if stderr_tail.strip():
                log(f"  stderr: {stderr_tail}", level="WARN")
        return r
    except subprocess.TimeoutExpired:
        log(f"  TIMEOUT after {timeout}s", level="ERROR")
        return subprocess.CompletedProcess(cmd, 124, "", "timeout")
    except Exception as e:
        log(f"  EXCEPTION: {e}", level="ERROR")
        return subprocess.CompletedProcess(cmd, 1, "", str(e))


def get_python():
    """Return path to venv python (falls back to system)."""
    vp = VENV_DIR / "bin" / "python"
    if vp.exists():
        return str(vp)
    return sys.executable


def get_pip():
    """Return path to venv pip (falls back to system)."""
    vp = VENV_DIR / "bin" / "pip"
    if vp.exists():
        return str(vp)
    return f"{sys.executable} -m pip"


def check_installed(pkg_import):
    """Check if a package is importable in the venv"""
    r = run_cmd([get_python(), "-c", f"import {pkg_import}; print('OK')"],
                timeout=30)
    return r.returncode == 0 and "OK" in (r.stdout or "")


# phase 1: installation

def create_venv():
    """Create a Python virtual environment with system packages"""
    if (VENV_DIR / "bin" / "python").exists():
        r = run_cmd([str(VENV_DIR / "bin" / "python"), "--version"])
        if r.returncode == 0:
            log(f"Reusing venv: {(r.stdout or '').strip()}")
            return True

    log(f"Creating venv at {VENV_DIR}...")
    r = run_cmd([sys.executable, "-m", "venv", str(VENV_DIR),
                 "--system-site-packages"])
    if r.returncode != 0:
        log("venv creation failed, trying without --system-site-packages")
        r = run_cmd([sys.executable, "-m", "venv", str(VENV_DIR)])
    return r.returncode == 0


def install_pytorch():
    """Install PyTorch with CUDA support"""
    python = get_python()
    if check_installed("torch"):
        r = run_cmd([python, "-c",
                     "import torch; print(torch.__version__, torch.cuda.is_available())"])
        log(f"PyTorch already installed: {(r.stdout or '').strip()}")
        return True

    log("Installing PyTorch...")
    pip = get_pip()
    pip_cmd = [pip] if not isinstance(pip, str) or " " not in pip else pip.split()

    # RTX 5080 (Blackwell sm_120) needs cu128 nightly
    for idx_url in [
        "https://download.pytorch.org/whl/nightly/cu128",
        "https://download.pytorch.org/whl/cu124",
        "https://download.pytorch.org/whl/cu121",
    ]:
        tag = idx_url.rsplit("/", 1)[-1]
        log(f"  Trying PyTorch with {tag}...")
        extra = ["--pre"] if "nightly" in idx_url else []
        r = run_cmd(
            pip_cmd + ["install"] + extra +
            ["torch", "torchvision", "torchaudio",
             "--index-url", idx_url],
            timeout=1200,
        )
        if r.returncode == 0 and check_installed("torch"):
            # verify CUDA actually works (not just importable)
            r2 = run_cmd([python, "-c",
                          "import torch; x=torch.randn(2,device='cuda'); "
                          "print(torch.__version__, 'CUDA_OK')"],
                         timeout=30)
            if "CUDA_OK" in (r2.stdout or ""):
                log(f"PyTorch installed: {(r2.stdout or '').strip()}")
                return True
            log(f"  PyTorch {tag} imported but CUDA failed, trying next...")

    log("All PyTorch install attempts failed", level="ERROR")
    return False


def install_base_deps():
    """Install evo, gdown, and other common dependencies"""
    pip = get_pip()
    pip_cmd = [pip] if not isinstance(pip, str) or " " not in pip else pip.split()

    deps = ["evo", "gdown", "tqdm", "scipy", "pyyaml",
            "tensorboard", "opencv-python", "einops", "numba", "pypose", "yacs"]
    run_cmd(pip_cmd + ["install", "--quiet"] + deps, timeout=300)


def install_droid_slam():
    """Clone and build DROID-SLAM. Returns True on success"""
    log("="*60)
    log("Installing DROID-SLAM")
    log("="*60)

    droid_dir = INSTALL_DIR / "DROID-SLAM"
    INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    python = get_python()
    pip = get_pip()
    pip_cmd = [pip] if not isinstance(pip, str) or " " not in pip else pip.split()

    # --- Clone ---
    if not (droid_dir / "droid_slam").exists():
        log("Cloning DROID-SLAM...")
        r = run_cmd(["git", "clone", "--recursive",
                     "https://github.com/princeton-vl/DROID-SLAM.git",
                     str(droid_dir)], timeout=300)
        if r.returncode != 0:
            log("Clone failed", level="ERROR")
            return False
    else:
        log("DROID-SLAM already cloned")

    # --- Install lietorch ---
    if not check_installed("lietorch"):
        log("Building lietorch from submodule...")
        lt_dir = droid_dir / "thirdparty" / "lietorch"
        r = run_cmd(pip_cmd + ["install", str(lt_dir)], timeout=600,
                    cwd=str(lt_dir))
        if r.returncode != 0:
            log("lietorch submodule failed, trying PyPI...")
            r = run_cmd(pip_cmd + ["install", "lietorch"], timeout=600)
        if r.returncode != 0:
            log("lietorch installation failed", level="ERROR")
            return False
    log("lietorch OK")

    # --- Install pytorch_scatter ---
    if not check_installed("torch_scatter"):
        log("Building pytorch_scatter...")
        ps_dir = droid_dir / "thirdparty" / "pytorch_scatter"
        if ps_dir.exists():
            r = run_cmd(pip_cmd + ["install", str(ps_dir)], timeout=600)
        else:
            r = subprocess.CompletedProcess([], 1)
        if r.returncode != 0:
            log("Trying torch-scatter from PyPI...")
            r = run_cmd(pip_cmd + ["install", "torch-scatter"], timeout=600)
        if r.returncode != 0:
            log("pytorch_scatter failed (may still work)", level="WARN")
    else:
        log("pytorch_scatter OK")

    # --- Build droid_backends (must run setup.py from DROID-SLAM dir) ---
    so_file = droid_dir / "droid_backends.cpython-312-x86_64-linux-gnu.so"
    if so_file.exists():
        log(f"droid_backends already built: {so_file}")
    else:
        log("Building droid_backends...")
        r = run_cmd([python, "setup.py", "build_ext", "--inplace"],
                    timeout=600, cwd=str(droid_dir))
        if r.returncode != 0:
            log("droid_backends build failed", level="ERROR")
            return False
    log("droid_backends OK")

    # --- Download weights ---
    weights = droid_dir / "droid.pth"
    if not weights.exists():
        log("Downloading DROID-SLAM weights...")
        # try gdown
        r = run_cmd([python, "-m", "gdown",
                     "1PpqVt1H4maBa_GbPJp4NwxRsd9jk-elh",
                     "-O", str(weights)], timeout=300)
        if not weights.exists():
            # try tools/download_model.sh
            dl_script = droid_dir / "tools" / "download_model.sh"
            if dl_script.exists():
                run_cmd(["bash", str(dl_script)], timeout=300,
                        cwd=str(droid_dir))
    if weights.exists():
        log(f"Weights: {weights} ({weights.stat().st_size/1e6:.0f} MB)")
    else:
        log("Weights download failed, DROID-SLAM will not run", level="ERROR")
        return False

    log("DROID-SLAM installation complete")
    return True


def install_dpvo():
    """Clone and build DPVO (also provides DPV-SLAM). Returns True on success"""
    log("="*60)
    log("Installing DPVO / DPV-SLAM")
    log("="*60)

    dpvo_dir = INSTALL_DIR / "DPVO"
    INSTALL_DIR.mkdir(parents=True, exist_ok=True)
    python = get_python()
    pip = get_pip()
    pip_cmd = [pip] if not isinstance(pip, str) or " " not in pip else pip.split()

    # --- Clone ---
    if not (dpvo_dir / "dpvo").exists():
        log("Cloning DPVO...")
        r = run_cmd(["git", "clone", "--recursive",
                     "https://github.com/princeton-vl/DPVO.git",
                     str(dpvo_dir)], timeout=300)
        if r.returncode != 0:
            log("Clone failed", level="ERROR")
            return False
    else:
        log("DPVO already cloned")

    # --- Download Eigen ---
    eigen_dir = dpvo_dir / "thirdparty" / "eigen-3.4.0"
    if not eigen_dir.exists():
        log("Downloading Eigen 3.4.0...")
        run_cmd(["wget", "-q",
                 "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip",
                 "-O", str(dpvo_dir / "eigen.zip")], timeout=120)
        run_cmd(["unzip", "-q", "-o", str(dpvo_dir / "eigen.zip"),
                 "-d", str(dpvo_dir / "thirdparty")], timeout=60)
    if eigen_dir.exists():
        log("Eigen OK")

    # --- Install lietorch (shared with DROID-SLAM) ---
    if not check_installed("lietorch"):
        lt_dir = dpvo_dir / "thirdparty" / "lietorch"
        if lt_dir.exists():
            log("Building lietorch from DPVO submodule...")
            run_cmd(pip_cmd + ["install", str(lt_dir)], timeout=600)
        if not check_installed("lietorch"):
            log("Trying lietorch from PyPI...")
            run_cmd(pip_cmd + ["install", "lietorch"], timeout=600)

    # --- Patch DPVO for PyTorch 2.7+ API (deprecated .type() removal) ---
    log("Patching DPVO for modern PyTorch API...")
    import re as _re
    for cu_file in dpvo_dir.rglob("*.cu"):
        txt = cu_file.read_text()
        # only replace tensor.type() inside AT_DISPATCH macros, not device.type()
        new_txt = _re.sub(
            r'(AT_DISPATCH_\w+\([^,]+)\.type\(\)',
            r'\1.scalar_type()',
            txt,
        )
        new_txt = _re.sub(
            r'(DISPATCH_GROUP_AND_FLOATING_TYPES\(\w+,\s*\w+)\.type\(\)',
            r'\1.scalar_type()',
            new_txt,
        )
        if new_txt != txt:
            cu_file.write_text(new_txt)
            log(f"  Patched {cu_file.name}")
    for cpp_file in dpvo_dir.rglob("*.cpp"):
        txt = cpp_file.read_text()
        new_txt = _re.sub(
            r'(DISPATCH_GROUP_AND_FLOATING_TYPES\(\w+,\s*\w+)\.type\(\)',
            r'\1.scalar_type()',
            txt,
        )
        if new_txt != txt:
            cpp_file.write_text(new_txt)
            log(f"  Patched {cpp_file.name}")
    # fix dispatch.h: ::detail::scalar_type() doesn't accept at::ScalarType
    disp_h = dpvo_dir / "dpvo" / "lietorch" / "include" / "dispatch.h"
    if disp_h.exists():
        txt = disp_h.read_text()
        txt = txt.replace(
            "at::ScalarType _st = ::detail::scalar_type(the_type);",
            "at::ScalarType _st = the_type;",
        )
        disp_h.write_text(txt)
        log("  Patched dispatch.h")

    # --- Build DPVO (must run setup.py from DPVO dir) ---
    so_files = list(dpvo_dir.glob("*.so"))
    if len(so_files) >= 3:
        log(f"DPVO already built: {[s.name for s in so_files]}")
    else:
        log("Building DPVO package...")
        r = run_cmd([python, "setup.py", "build_ext", "--inplace"],
                    timeout=600, cwd=str(dpvo_dir))
        if r.returncode != 0:
            log("DPVO build failed", level="ERROR")
            return False
    log("DPVO build OK")

    # --- Download weights ---
    log("Downloading DPVO weights...")
    dpvo_pth = dpvo_dir / "dpvo.pth"
    if dpvo_pth.exists():
        log(f"Weights already present: {dpvo_pth} ({dpvo_pth.stat().st_size // 1024**2} MB)")
    else:
        # only download the weight file, NOT the full dataset script
        # download_models_and_data.sh also downloads TartanAir + movies (huge, unnecessary)
        log("Downloading dpvo.pth from Dropbox...")
        run_cmd(["wget", "-q", "-O", str(dpvo_pth),
                 "https://www.dropbox.com/s/nap0u8zslspdwm4/models.zip"],
                timeout=300, cwd=str(dpvo_dir))
        if not dpvo_pth.exists():
            # fallback: try the full download script
            dl_script = dpvo_dir / "download_models_and_data.sh"
            if dl_script.exists():
                run_cmd(["bash", str(dl_script)], timeout=600, cwd=str(dpvo_dir))

    # find any .pth file
    pth_files = list(dpvo_dir.rglob("*.pth"))
    if pth_files:
        log(f"Weights found: {[str(p.relative_to(dpvo_dir)) for p in pth_files[:5]]}")
    else:
        # try manual gdown
        log("Trying manual weight download...")
        run_cmd([python, "-m", "gdown", "--fuzzy",
                 "https://drive.google.com/drive/folders/1kGbPlkKdukR4EY9SUHcN7qFelHXW3Zzl",
                 "-O", str(dpvo_dir / "models/")], timeout=300)
        pth_files = list(dpvo_dir.rglob("*.pth"))

    if not pth_files:
        log("No weights found, DPVO will not run", level="ERROR")
        return False

    log("DPVO / DPV-SLAM installation complete")
    return True


def phase1_install():
    """Phase 1: Install all methods"""
    log("=" * 60)
    log("PHASE 1: INSTALLATION")
    log("=" * 60)

    installed = {}

    # base environment
    if not create_venv():
        log("Failed to create venv, using system Python", level="WARN")
    if not install_pytorch():
        log("PyTorch not installed, no methods will work", level="ERROR")
        return installed
    install_base_deps()

    # DROID-SLAM
    try:
        installed["droid_slam"] = install_droid_slam()
    except Exception as e:
        log(f"DROID-SLAM install error: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")
        installed["droid_slam"] = False

    # DPVO (also gives us DPV-SLAM)
    try:
        installed["dpvo"] = install_dpvo()
    except Exception as e:
        log(f"DPVO install error: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")
        installed["dpvo"] = False

    installed["dpv_slam"] = installed.get("dpvo", False)

    log(f"Installation summary: {installed}")
    return installed


# phase 2: data preparation

def _convert_one_image(args):
    """Worker: convert one TIFF to PNG at 808x616. (for multiprocessing)"""
    src, dst = args
    try:
        img = cv2.imread(str(src), cv2.IMREAD_COLOR)
        if img is None:
            return False
        img = cv2.resize(img, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(dst), img)
        return True
    except Exception:
        return False


def prepare_images(session):
    """convert Ladybug3 TIFF images to 808x616 PNG for one session"""
    src_dir = DATA_DIR / "images" / session / "lb3" / "Cam0"
    dst_dir = PREPARED_BASE / session

    if not src_dir.exists():
        log(f"Source images not found: {src_dir}", level="ERROR")
        return None

    # check if already prepared
    if dst_dir.exists():
        n_existing = len(list(dst_dir.glob("*.png")))
        n_source = len(list(src_dir.glob("*.tiff")))
        if n_existing >= n_source * 0.95:
            log(f"Images already prepared: {n_existing} PNGs in {dst_dir}")
            return dst_dir

    dst_dir.mkdir(parents=True, exist_ok=True)

    tiff_files = sorted(src_dir.glob("*.tiff"))
    log(f"Converting {len(tiff_files)} TIFF images for {session}...")

    # build work list (use microsecond timestamp as filename)
    work = []
    for f in tiff_files:
        ts_us = f.stem  # e.g. 1335704127712909
        out = dst_dir / f"{ts_us}.png"
        if not out.exists():
            work.append((f, out))

    if not work:
        log(f"All images already converted for {session}")
        return dst_dir

    log(f"  {len(work)} images to convert ({len(tiff_files) - len(work)} cached)")

    n_workers = min(8, mp.cpu_count())
    done = 0
    t0 = time.time()

    with mp.Pool(n_workers) as pool:
        for ok in pool.imap_unordered(_convert_one_image, work, chunksize=50):
            done += 1
            if done % 5000 == 0:
                elapsed = time.time() - t0
                rate = done / elapsed
                eta = (len(work) - done) / rate if rate > 0 else 0
                log(f"  {done}/{len(work)} converted "
                    f"({rate:.0f} img/s, ETA {eta:.0f}s)")

    elapsed = time.time() - t0
    n_total = len(list(dst_dir.glob("*.png")))
    log(f"  Done: {n_total} PNGs in {elapsed:.0f}s")
    return dst_dir


def create_calibration_file():
    """create calibration file: fx fy cx cy (one line, space-separated)"""
    calib_path = RESULTS_DIR / "calib.txt"
    calib_path.parent.mkdir(parents=True, exist_ok=True)
    with open(calib_path, "w") as f:
        f.write(f"{FX} {FY} {CX} {CY}\n")
    return calib_path


def prepare_ground_truth_tum(session):
    """convert NCLT ground truth CSV to TUM format"""
    gt_csv = DATA_DIR / "ground_truth" / f"groundtruth_{session}.csv"
    gt_tum = RESULTS_DIR / "ground_truth" / f"gt_{session}.tum"

    if gt_tum.exists() and gt_tum.stat().st_size > 1000:
        log(f"Ground truth TUM already exists: {gt_tum}")
        return gt_tum

    gt_tum.parent.mkdir(parents=True, exist_ok=True)
    log(f"Converting ground truth for {session}...")

    data = np.loadtxt(str(gt_csv), delimiter=",")
    # columns: utime, x, y, z, qx, qy, qz
    utimes = data[:, 0]
    xyz = data[:, 1:4]
    qxyz = data[:, 4:7]

    # compute qw
    qw_sq = 1.0 - np.sum(qxyz ** 2, axis=1)
    qw = np.sqrt(np.maximum(qw_sq, 0.0))

    # drop NaN rows
    valid = np.isfinite(qw) & np.all(np.isfinite(xyz), axis=1)
    utimes, xyz, qxyz, qw = utimes[valid], xyz[valid], qxyz[valid], qw[valid]

    # write TUM format
    with open(gt_tum, "w") as f:
        for i in range(len(utimes)):
            ts_s = utimes[i] / 1e6
            x, y, z = xyz[i]
            qx, qy, qz = qxyz[i]
            f.write(f"{ts_s:.6f} {x:.6f} {y:.6f} {z:.6f} "
                    f"{qx:.9f} {qy:.9f} {qz:.9f} {qw[i]:.9f}\n")

    log(f"  {len(utimes)} poses -> {gt_tum}")
    return gt_tum


def phase2_prepare():
    """Phase 2: Prepare images, calibration, ground truth"""
    log("=" * 60)
    log("PHASE 2: DATA PREPARATION")
    log("=" * 60)

    image_dirs = {}
    gt_files = {}

    calib = create_calibration_file()
    log(f"Calibration: {calib} (fx={FX}, fy={FY}, cx={CX}, cy={CY})")

    for session in SESSIONS:
        name = SESSION_NAMES[session]
        log(f"--- Preparing {name} ({session}) ---")

        # images
        img_dir = prepare_images(session)
        if img_dir:
            image_dirs[session] = img_dir
        else:
            log(f"No images for {session}", level="ERROR")

        # ground truth
        gt = prepare_ground_truth_tum(session)
        if gt:
            gt_files[session] = gt

    return image_dirs, gt_files, calib


# phase 3: run methods

def find_dpvo_weights():
    """Find the best DPVO model weights file"""
    dpvo_dir = INSTALL_DIR / "DPVO"
    candidates = [
        dpvo_dir / "dpvo.pth",
        dpvo_dir / "models" / "dpvo.pth",
    ]
    # search for any .pth
    for c in candidates:
        if c.exists():
            return str(c)
    for p in sorted(dpvo_dir.rglob("*.pth")):
        return str(p)
    return str(dpvo_dir / "dpvo.pth")  # hope for the best


def run_single_method(method_key, session, image_dir, calib_file):
    """run a single method on a single session"""
    method = METHODS[method_key]
    name = method["name"]
    timeout_s = method["timeout_h"] * 3600
    sname = SESSION_NAMES[session]

    out_dir = RESULTS_DIR / method_key / session
    out_dir.mkdir(parents=True, exist_ok=True)

    traj_file = out_dir / "trajectory.txt"
    stats_file = out_dir / "stats.json"
    log_file = out_dir / "method_log.txt"

    log(f"--- Running {name} on {sname} ({session}) ---")
    log(f"  Images: {image_dir}")
    log(f"  Output: {out_dir}")

    # skip if trajectory already exists with poses
    if traj_file.exists() and stats_file.exists():
        try:
            with open(traj_file) as f:
                n_lines = sum(1 for line in f if line.strip())
            if n_lines > 0:
                import json as _json
                with open(stats_file) as f:
                    cached_stats = _json.load(f)
                log(f"  SKIPPING, already have {n_lines} poses from previous run")
                return True, cached_stats
        except Exception:
            pass  # re-run if files are corrupted

    python = get_python()

    if method_key == "droid_slam":
        runner_path = INSTALL_DIR / "run_droid.py"
        with open(runner_path, "w") as f:
            f.write(DROID_RUNNER_SCRIPT)

        config = dict(
            imagedir=str(image_dir),
            calib=str(calib_file),
            output_trajectory=str(traj_file),
            output_stats=str(stats_file),
            weights=str(INSTALL_DIR / "DROID-SLAM" / "droid.pth"),
            droid_dir=str(INSTALL_DIR / "DROID-SLAM"),
            stride=1,
            buffer=2048,
        )

        r = run_cmd(
            [python, str(runner_path), json.dumps(config)],
            timeout=timeout_s,
            cwd=str(INSTALL_DIR / "DROID-SLAM"),
            capture=False,  # stream output to console
        )

    elif method_key in ("dpvo", "dpv_slam"):
        runner_path = INSTALL_DIR / "run_dpvo.py"
        with open(runner_path, "w") as f:
            f.write(DPVO_RUNNER_SCRIPT)

        config = dict(
            imagedir=str(image_dir),
            calib=str(calib_file),
            output_trajectory=str(traj_file),
            output_stats=str(stats_file),
            weights=find_dpvo_weights(),
            dpvo_dir=str(INSTALL_DIR / "DPVO"),
            method="dpv_slam" if method_key == "dpv_slam" else "dpvo",
            stride=1,
        )

        r = run_cmd(
            [python, str(runner_path), json.dumps(config)],
            timeout=timeout_s,
            cwd=str(INSTALL_DIR / "DPVO"),
            capture=False,
        )

    else:
        log(f"Unknown method: {method_key}", level="ERROR")
        return False, {}

    # check results
    stats = {}
    if stats_file.exists():
        try:
            with open(stats_file) as f:
                stats = json.load(f)
        except Exception:
            pass

    if traj_file.exists() and traj_file.stat().st_size > 0:
        n_lines = sum(1 for _ in open(traj_file))
        log(f"  Result: {n_lines} poses, "
            f"runtime={stats.get('runtime_total_s', stats.get('runtime_s', '?'))}s")
        return True, stats
    else:
        log(f"  {name} produced no trajectory", level="WARN")
        return False, stats


def phase3_run(installed, image_dirs, calib_file):
    """phase 3: Run all installed methods on all sessions"""
    log("=" * 60)
    log("PHASE 3: RUNNING METHODS")
    log("=" * 60)

    all_stats = {}

    for method_key in METHODS:
        if not installed.get(method_key, False):
            log(f"Skipping {METHODS[method_key]['name']} (not installed)")
            continue

        for session in SESSIONS:
            if session not in image_dirs:
                log(f"Skipping {session} (no images)")
                continue

            key = (method_key, session)
            try:
                success, stats = run_single_method(
                    method_key, session, image_dirs[session], calib_file
                )
                stats["success"] = success
                all_stats[key] = stats
            except Exception as e:
                log(f"{METHODS[method_key]['name']} on {session} CRASHED: {e}",
                    level="ERROR")
                log(traceback.format_exc(), level="ERROR")
                all_stats[key] = {"success": False, "error": str(e)}

            # save intermediate results
            with open(RESULTS_DIR / "intermediate_stats.json", "w") as f:
                json.dump({f"{k[0]}_{k[1]}": v for k, v in all_stats.items()},
                          f, indent=2)

    return all_stats


# phase 4: evaluation

def umeyama_alignment(src, dst, with_scale=True):
    """sim(3) alignment using Umeyama's method"""
    assert src.shape == dst.shape and src.shape[1] == 3
    n = src.shape[0]

    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)

    src_c = src - mu_s
    dst_c = dst - mu_d

    var_s = np.sum(src_c ** 2) / n

    cov = dst_c.T @ src_c / n
    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / var_s if with_scale else 1.0
    t = mu_d - s * R @ mu_s

    return s, R, t


def sync_trajectories(est, gt, max_dt=0.15):
    """match estimated poses to ground truth by nearest timestamp"""
    est_s, gt_s = [], []
    gt_times = gt[:, 0]

    for row in est:
        diffs = np.abs(gt_times - row[0])
        idx = np.argmin(diffs)
        if diffs[idx] < max_dt:
            est_s.append(row)
            gt_s.append(gt[idx])

    if not est_s:
        return np.zeros((0, 8)), np.zeros((0, 8))
    return np.array(est_s), np.array(gt_s)


def evaluate_trajectory(method_key, session, gt_tum_path):
    """evaluate a single method/session trajectory"""
    method_name = METHODS[method_key]["name"]
    sname = SESSION_NAMES[session]
    traj_path = RESULTS_DIR / method_key / session / "trajectory.txt"
    out_dir = RESULTS_DIR / method_key / session

    result = {"method": method_name, "session": sname, "status": "failed"}

    if not traj_path.exists() or traj_path.stat().st_size == 0:
        result["reason"] = "no trajectory file"
        return result

    try:
        est = np.loadtxt(str(traj_path))
        gt = np.loadtxt(str(gt_tum_path))
    except Exception as e:
        result["reason"] = f"load error: {e}"
        return result

    if est.ndim != 2 or est.shape[1] < 7:
        result["reason"] = f"bad shape: {est.shape}"
        return result

    # pad to 8 columns (add qw=1 if missing)
    if est.shape[1] == 7:
        est = np.column_stack([est, np.ones(len(est))])
    if gt.shape[1] == 7:
        gt = np.column_stack([gt, np.ones(len(gt))])

    # sync
    est_sync, gt_sync = sync_trajectories(est, gt, max_dt=0.15)
    if len(est_sync) < 10:
        result["reason"] = f"only {len(est_sync)} matched poses"
        return result

    # sim(3) alignment
    s, R, t = umeyama_alignment(est_sync[:, 1:4], gt_sync[:, 1:4], with_scale=True)
    est_aligned = s * (R @ est_sync[:, 1:4].T).T + t

    # ATE
    errors = np.linalg.norm(est_aligned - gt_sync[:, 1:4], axis=1)
    ate_rmse = float(np.sqrt(np.mean(errors ** 2)))
    ate_mean = float(np.mean(errors))
    ate_median = float(np.median(errors))

    # Path lengths
    est_length = float(np.sum(np.linalg.norm(np.diff(est_aligned, axis=0), axis=1)))
    gt_length = float(np.sum(np.linalg.norm(np.diff(gt_sync[:, 1:4], axis=0), axis=1)))

    # full GT length for coverage
    gt_full_length = float(np.sum(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1)))

    result.update(dict(
        status="success",
        n_poses_total=int(len(est)),
        n_poses_synced=int(len(est_sync)),
        ate_rmse=ate_rmse,
        ate_mean=ate_mean,
        ate_median=ate_median,
        scale=float(s),
        est_path_length_m=est_length,
        gt_synced_length_m=gt_length,
        gt_full_length_m=gt_full_length,
        coverage_pct=100.0 * gt_length / gt_full_length if gt_full_length > 0 else 0,
    ))

    np.savetxt(
        str(out_dir / "trajectory_aligned.txt"),
        np.column_stack([est_sync[:, 0:1], est_aligned, est_sync[:, 4:]]),
        fmt="%.6f",
    )

    # save eval results
    with open(out_dir / "eval_results.json", "w") as f:
        json.dump(result, f, indent=2)

    log(f"  {method_name} {sname}: ATE={ate_rmse:.1f}m, "
        f"scale={s:.2f}, poses={len(est_sync)}, "
        f"coverage={result['coverage_pct']:.1f}%")

    return result


def generate_plots(all_results):
    """generate comparison plots"""
    log("Generating plots...")
    colors = {"droid_slam": "#e74c3c", "dpvo": "#3498db", "dpv_slam": "#2ecc71"}

    # ------------------------------------------------------------------
    # plot 1: Trajectory overlays (one subplot per session)
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    for idx, session in enumerate(SESSIONS):
        ax = axes[idx]
        sname = SESSION_NAMES[session]

        # ground truth
        gt_path = RESULTS_DIR / "ground_truth" / f"gt_{session}.tum"
        if gt_path.exists():
            gt = np.loadtxt(str(gt_path))
            ax.plot(gt[:, 1], gt[:, 2], "k-", alpha=0.2, lw=0.5,
                    label="Ground Truth")

        # methods
        for mk in METHODS:
            res = all_results.get((mk, session), {})
            if res.get("status") != "success":
                continue
            tpath = RESULTS_DIR / mk / session / "trajectory_aligned.txt"
            if not tpath.exists():
                continue
            traj = np.loadtxt(str(tpath))
            ate = res["ate_rmse"]
            lbl = f'{METHODS[mk]["name"]} (ATE {ate:.1f}m)'
            ax.plot(traj[:, 1], traj[:, 2], color=colors[mk], lw=1.5,
                    alpha=0.85, label=lbl)

        ax.set_title(f"{sname.capitalize()} ({session})", fontsize=13)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.legend(fontsize=9, loc="best")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    fig.suptitle("Visual SLAM Trajectories vs Ground Truth (NCLT)", fontsize=14)
    plt.tight_layout()
    plt.savefig(str(RESULTS_DIR / "trajectory_all_methods.png"), dpi=150)
    plt.close()

    # ------------------------------------------------------------------
    # plot 2: ATE bar chart
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 6))
    method_names = []
    spring_ate, summer_ate = [], []

    for mk in METHODS:
        sr = all_results.get((mk, "2012-04-29"), {})
        su = all_results.get((mk, "2012-08-04"), {})
        if sr.get("status") == "success" or su.get("status") == "success":
            method_names.append(METHODS[mk]["name"])
            spring_ate.append(sr.get("ate_rmse", 0) if sr.get("status") == "success" else 0)
            summer_ate.append(su.get("ate_rmse", 0) if su.get("status") == "success" else 0)

    # add LiDAR reference
    method_names.append("LiDAR ICP\n(reference)")
    spring_ate.append(LIDAR_ATE["2012-04-29"])
    summer_ate.append(LIDAR_ATE["2012-08-04"])

    if method_names:
        x = np.arange(len(method_names))
        w = 0.35
        ax.bar(x - w / 2, spring_ate, w, label="Spring", color="#2ecc71", alpha=0.8)
        ax.bar(x + w / 2, summer_ate, w, label="Summer", color="#e67e22", alpha=0.8)
        ax.set_ylabel("ATE RMSE (m)")
        ax.set_title("Absolute Trajectory Error -All Methods")
        ax.set_xticks(x)
        ax.set_xticklabels(method_names, fontsize=10)
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")

        # value labels
        for i, (sv, suv) in enumerate(zip(spring_ate, summer_ate)):
            if sv > 0:
                ax.text(i - w / 2, sv + 2, f"{sv:.1f}", ha="center", fontsize=8)
            if suv > 0:
                ax.text(i + w / 2, suv + 2, f"{suv:.1f}", ha="center", fontsize=8)

    plt.tight_layout()
    plt.savefig(str(RESULTS_DIR / "ate_comparison.png"), dpi=150)
    plt.close()

    # ------------------------------------------------------------------
    # plot 3: Tracking success rate
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 6))
    method_names_t = []
    spring_rate, summer_rate = [], []

    for mk in METHODS:
        sr = all_results.get((mk, "2012-04-29"), {})
        su = all_results.get((mk, "2012-08-04"), {})
        if sr or su:
            method_names_t.append(METHODS[mk]["name"])
            n_spring_img = len(list((PREPARED_BASE / "2012-04-29").glob("*.png"))) if (PREPARED_BASE / "2012-04-29").exists() else 1
            n_summer_img = len(list((PREPARED_BASE / "2012-08-04").glob("*.png"))) if (PREPARED_BASE / "2012-08-04").exists() else 1
            sp = 100 * sr.get("n_poses_synced", 0) / max(n_spring_img, 1) if sr.get("status") == "success" else 0
            su_r = 100 * su.get("n_poses_synced", 0) / max(n_summer_img, 1) if su.get("status") == "success" else 0
            spring_rate.append(sp)
            summer_rate.append(su_r)

    if method_names_t:
        x = np.arange(len(method_names_t))
        w = 0.35
        ax.bar(x - w / 2, spring_rate, w, label="Spring", color="#2ecc71", alpha=0.8)
        ax.bar(x + w / 2, summer_rate, w, label="Summer", color="#e67e22", alpha=0.8)
        ax.set_ylabel("Tracking Success (%)")
        ax.set_title("Tracking Success Rate")
        ax.set_xticks(x)
        ax.set_xticklabels(method_names_t, fontsize=10)
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")
        ax.set_ylim(0, 105)

    plt.tight_layout()
    plt.savefig(str(RESULTS_DIR / "tracking_success_rate.png"), dpi=150)
    plt.close()

    # ------------------------------------------------------------------
    # plot 4: ATE over trajectory (per method, spring only)
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(12, 5))
    session = "2012-04-29"
    gt_path = RESULTS_DIR / "ground_truth" / f"gt_{session}.tum"
    if gt_path.exists():
        gt = np.loadtxt(str(gt_path))

    for mk in METHODS:
        res = all_results.get((mk, session), {})
        if res.get("status") != "success":
            continue
        tpath = RESULTS_DIR / mk / session / "trajectory_aligned.txt"
        if not tpath.exists():
            continue
        traj = np.loadtxt(str(tpath))
        est_sync, gt_sync = sync_trajectories(
            np.column_stack([traj, np.ones((len(traj), max(0, 8 - traj.shape[1])))]) if traj.shape[1] < 8 else traj,
            gt, max_dt=0.15
        )
        if len(est_sync) < 2:
            continue
        errors = np.linalg.norm(est_sync[:, 1:4] - gt_sync[:, 1:4], axis=1)
        dist_along = np.cumsum(np.r_[0, np.linalg.norm(np.diff(gt_sync[:, 1:4], axis=0), axis=1)])
        ax.plot(dist_along, errors, color=colors[mk], alpha=0.8,
                label=f'{METHODS[mk]["name"]}')

    ax.set_xlabel("Distance along GT (m)")
    ax.set_ylabel("Position error (m)")
    ax.set_title("ATE vs Distance -Spring (2012-04-29)")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(str(RESULTS_DIR / "ate_over_distance_spring.png"), dpi=150)
    plt.close()

    log(f"Plots saved to {RESULTS_DIR}")


def phase4_evaluate(gt_files):
    """phase 4: Evaluate all trajectories and generate plots"""
    log("=" * 60)
    log("PHASE 4: EVALUATION")
    log("=" * 60)

    all_results = {}

    for method_key in METHODS:
        for session in SESSIONS:
            gt_path = gt_files.get(session)
            if not gt_path:
                log(f"No GT for {session}", level="WARN")
                continue
            try:
                res = evaluate_trajectory(method_key, session, gt_path)
                all_results[(method_key, session)] = res
            except Exception as e:
                log(f"Eval {method_key}/{session} failed: {e}", level="ERROR")
                log(traceback.format_exc(), level="ERROR")
                all_results[(method_key, session)] = {
                    "status": "failed", "reason": str(e)
                }

    # generate plots
    try:
        generate_plots(all_results)
    except Exception as e:
        log(f"Plot generation failed: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")

    return all_results


# phase 5: save results

def save_summary(all_results, all_stats):
    """Save summary tables to text and JSON"""
    summary_path = RESULTS_DIR / "summary.txt"

    lines = [
        "=" * 70,
        "VISUAL SLAM BENCHMARK -NCLT DATASET",
        f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        f"Camera: Ladybug3 Cam0, f=221, 808x616",
        "=" * 70,
        "",
        f"{'Method':<14} {'Session':<10} {'Poses':>8} {'ATE RMSE':>10} "
        f"{'Scale':>7} {'Coverage':>10} {'Runtime':>10} {'GPU':>7}",
        "-" * 70,
    ]

    for mk in METHODS:
        for session in SESSIONS:
            sname = SESSION_NAMES[session]
            res = all_results.get((mk, session), {})
            st = all_stats.get((mk, session), {})

            name = METHODS[mk]["name"]
            n_poses = res.get("n_poses_synced", 0)
            ate = f"{res['ate_rmse']:.1f}m" if res.get("status") == "success" else "FAILED"
            scale = f"{res['scale']:.2f}" if res.get("status") == "success" else "-"
            cov = f"{res.get('coverage_pct', 0):.1f}%" if res.get("status") == "success" else "-"
            rt = st.get("runtime_total_s", st.get("runtime_s", 0))
            runtime = f"{rt / 60:.0f}min" if rt else "-"
            gpu = f"{st.get('gpu_memory_peak_gb', 0):.1f}GB" if st.get("gpu_memory_peak_gb") else "-"

            lines.append(
                f"{name:<14} {sname:<10} {n_poses:>8} {ate:>10} "
                f"{scale:>7} {cov:>10} {runtime:>10} {gpu:>7}"
            )

    lines.extend([
        "-" * 70,
        "",
        "Reference LiDAR ICP results:",
        f"  Spring: {LIDAR_ATE['2012-04-29']:.1f}m ATE (100% coverage)",
        f"  Summer: {LIDAR_ATE['2012-08-04']:.1f}m ATE (100% coverage)",
        "",
    ])

    summary = "\n".join(lines)
    with open(summary_path, "w") as f:
        f.write(summary)

    log(f"\n{summary}")

    # JSON summary
    json_summary = {}
    for mk in METHODS:
        for session in SESSIONS:
            key = f"{mk}_{session}"
            res = all_results.get((mk, session), {})
            st = all_stats.get((mk, session), {})
            json_summary[key] = {**res, **st}

    with open(RESULTS_DIR / "summary.json", "w") as f:
        json.dump(json_summary, f, indent=2)


def update_changelog(all_results, all_stats):
    """Append experiment results to CHANGELOG.md"""
    changelog = BASE_DIR / "CHANGELOG.md"
    if not changelog.exists():
        log("CHANGELOG.md not found", level="WARN")
        return

    # build results table
    table = []
    table.append("| Method | Session | Poses | ATE RMSE | Scale | Coverage | Runtime | GPU |")
    table.append("|--------|---------|-------|----------|-------|----------|---------|-----|")

    for mk in METHODS:
        for session in SESSIONS:
            sname = SESSION_NAMES[session]
            res = all_results.get((mk, session), {})
            st = all_stats.get((mk, session), {})

            name = METHODS[mk]["name"]
            n_sync = res.get("n_poses_synced", 0)
            n_total = res.get("n_poses_total", 0)
            ate = f"{res['ate_rmse']:.1f}m" if res.get("status") == "success" else "FAILED"
            scale = f"{res['scale']:.2f}" if res.get("status") == "success" else "-"
            cov = f"{res.get('coverage_pct', 0):.1f}%" if res.get("status") == "success" else "-"
            rt = st.get("runtime_total_s", st.get("runtime_s", 0))
            runtime = f"{rt / 60:.0f}min" if rt else "-"
            gpu = f"{st.get('gpu_memory_peak_gb', 0):.1f}GB" if st.get("gpu_memory_peak_gb") else "-"

            table.append(f"| {name} | {sname} | {n_sync}/{n_total} | {ate} | "
                         f"{scale} | {cov} | {runtime} | {gpu} |")

    # count successes/failures
    n_success = sum(1 for r in all_results.values() if r.get("status") == "success")
    n_total = len(all_results)

    entry = f"""
### Experiment 0.4: Deep Visual SLAM Benchmark
- **Date:** {datetime.now().strftime('%Y-%m-%d')}
- **Sessions:** 2012-04-29 (spring), 2012-08-04 (summer)
- **Methods:** DROID-SLAM, DPVO (visual odometry), DPV-SLAM (DPVO + loop closure)
- **Camera:** Ladybug3 Cam0, f=221, cx=404, cy=308, 808x616 half-res (COLMAP calibrated)
- **Evaluation:** Sim(3) Umeyama alignment (monocular scale unknown), ATE RMSE
- **Results:** ({n_success}/{n_total} method/session combinations succeeded)

{chr(10).join(table)}

- **Reference LiDAR ICP:** Spring {LIDAR_ATE['2012-04-29']:.1f}m, Summer {LIDAR_ATE['2012-08-04']:.1f}m (100% coverage)
- **Files:** `results/week0_visual_slam/`
"""

    with open(changelog, "a") as f:
        f.write(entry)

    log("CHANGELOG.md updated")


def git_commit():
    """Stage results and create a git commit"""
    log("Creating git commit...")

    # add results
    run_cmd(["git", "add", "results/week0_visual_slam/", "CHANGELOG.md",
             "scripts/run_all_visual_slam.py"],
            cwd=str(BASE_DIR))

    # commit
    msg = ("Experiment 0.4: Deep Visual SLAM benchmark "
           "(DROID-SLAM, DPVO, DPV-SLAM)")
    r = run_cmd(
        ["git", "commit", "-m", msg],
        cwd=str(BASE_DIR),
    )
    if r.returncode == 0:
        log("Git commit created")
    else:
        log(f"Git commit failed: {(r.stderr or '')[:200]}", level="WARN")


def phase5_save(all_results, all_stats):
    """Phase 5: Save summary, update changelog, git commit"""
    log("=" * 60)
    log("PHASE 5: SAVING RESULTS")
    log("=" * 60)

    try:
        save_summary(all_results, all_stats)
    except Exception as e:
        log(f"Summary failed: {e}", level="ERROR")

    try:
        update_changelog(all_results, all_stats)
    except Exception as e:
        log(f"CHANGELOG update failed: {e}", level="ERROR")

    try:
        git_commit()
    except Exception as e:
        log(f"Git commit failed: {e}", level="ERROR")



def main():
    global START_TIME, LOG_FILE

    START_TIME = datetime.now()
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    LOG_FILE = str(RESULTS_DIR / "run_log.txt")

    log("=" * 60)
    log("OVERNIGHT VISUAL SLAM BENCHMARK")
    log(f"Start: {START_TIME.strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"Sessions: {SESSIONS}")
    log(f"Methods: {list(METHODS.keys())}")
    log("=" * 60)

    # phase 1: Install
    try:
        installed = phase1_install()
    except Exception as e:
        log(f"Phase 1 CRASHED: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")
        installed = {}

    any_installed = any(installed.values())
    if not any_installed:
        log("NO methods installed, nothing to run", level="ERROR")
        log("Check the log above for installation errors.")
        # still do data prep and save what we can
    # phase 2: Prepare data
    try:
        image_dirs, gt_files, calib_file = phase2_prepare()
    except Exception as e:
        log(f"Phase 2 CRASHED: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")
        image_dirs, gt_files, calib_file = {}, {}, None

    # phase 3: Run methods
    all_stats = {}
    if any_installed and image_dirs and calib_file:
        try:
            all_stats = phase3_run(installed, image_dirs, calib_file)
        except Exception as e:
            log(f"Phase 3 CRASHED: {e}", level="ERROR")
            log(traceback.format_exc(), level="ERROR")

    # phase 4: Evaluate
    all_results = {}
    if gt_files:
        try:
            all_results = phase4_evaluate(gt_files)
        except Exception as e:
            log(f"Phase 4 CRASHED: {e}", level="ERROR")
            log(traceback.format_exc(), level="ERROR")

    # phase 5: Save
    try:
        phase5_save(all_results, all_stats)
    except Exception as e:
        log(f"Phase 5 CRASHED: {e}", level="ERROR")
        log(traceback.format_exc(), level="ERROR")

    elapsed = datetime.now() - START_TIME
    log("=" * 60)
    log(f"BENCHMARK COMPLETE, elapsed {str(elapsed).split('.')[0]}")
    log(f"Results: {RESULTS_DIR}")
    log("=" * 60)


if __name__ == "__main__":
    main()
