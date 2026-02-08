#!/usr/bin/env python3
"""
Regenerate clean summary.txt and all_results.json for ROVER ORB-SLAM3 experiments.

Scans all 15 canonical recordings x 3 modes, reads eval_results.json and
orbslam3_log.txt to build a complete picture of successes and failures.
"""

import json
import os
import re
import statistics
from pathlib import Path

# =============================================================================
# =============================================================================

BASE_DIR = Path("/workspace/datasets/rover/results")

# the 15 canonical recording names (as specified in the experiment definition)
RECORDINGS = [
    "garden_large_autumn_2023-12-21",
    "garden_large_day_2024-05-29_1",
    "garden_large_dusk_2024-05-30_1",
    "garden_large_night-light_2024-05-30_2",
    "garden_large_night_2024-05-30_3",
    "garden_large_spring_2024-04-11",
    "garden_large_summer_2023-08-18",
    "garden_large_winter_2024-01-13",
    "park_autumn_2023-11-07",
    "park_day_2024-05-08",
    "park_dusk_2024-05-13_1",
    "park_night-light_2024-05-13_3",
    "park_night_2024-05-13_2",
    "park_spring_2024-04-14",
    "park_summer_2023-07-31",
]

MODES = ["stereo", "stereo_inertial", "rgbd"]

# map canonical recording names to actual directory names on disk# (some directory names differ from the canonical names)
DIR_MAPPING = {
    "garden_large_dusk_2024-05-30_1": "garden_large_dusk_2024-05-29_2",
    "garden_large_night_2024-05-30_3": "garden_large_night_2024-05-30_1",
    "park_night-light_2024-05-13_3": "park_night-light_2024-05-24_2",
}


def get_dir_name(recording: str) -> str:
    """Return the actual directory name for a canonical recording name"""
    return DIR_MAPPING.get(recording, recording)


def diagnose_log(log_path: Path) -> str:
    """Examine an orbslam3_log.txt for failure clues"""
    if not log_path.exists():
        return "no log"
    text = log_path.read_text(errors="replace")
    if not text.strip():
        return "empty log"

    # check for known failure patterns (order matters -- more specific first)
    if "Segmentation fault" in text:
        # also check KF count
        kf_matches = re.findall(r"Map 0 has (\d+) KFs", text)
        if kf_matches:
            last_kfs = int(kf_matches[-1])
            if last_kfs <= 1:
                return "tracking failure (1 KF) + segfault"
            return f"segfault ({last_kfs} KFs)"
        return "segfault"
    if "Aborted" in text and "system_error" in text:
        return "abort (system_error)"
    if "Aborted" in text and "Sophus" in text:
        return "abort (Sophus NaN)"
    if "Aborted" in text:
        return "abort"
    if "Killed" in text:
        return "killed (OOM?)"
    if "Timeout" in text.lower() or "timeout" in text.lower():
        return "timeout"

    # check if it only produced 1 KF (tracking failure)
    kf_matches = re.findall(r"Map 0 has (\d+) KFs", text)
    if kf_matches:
        last_kfs = int(kf_matches[-1])
        if last_kfs <= 1:
            return "tracking failure (1 KF)"
        if last_kfs < 10:
            return f"tracking failure ({last_kfs} KFs)"

    return "unknown"


def process_entry(recording: str, mode: str) -> dict:
    """Process a single recording+mode combination, returning a result dict"""
    dir_name = get_dir_name(recording)
    mode_dir = BASE_DIR / dir_name / mode
    eval_path = mode_dir / "eval_results.json"
    log_path = mode_dir / "orbslam3_log.txt"

    result = {
        "recording": recording,
        "mode": mode,
        "status": "FAIL",
        "fail_reason": None,
        "ate_rmse": None,
        "ate_mean": None,
        "ate_median": None,
        "ate_std": None,
        "ate_max": None,
        "num_estimated": None,
        "num_gt": None,
        "tracking_rate_pct": None,
        "sim3_scale": None,
    }

    # check if the mode directory exists at all
    if not mode_dir.exists():
        result["fail_reason"] = "no results directory"
        return result

    # try to read eval_results.json
    if eval_path.exists():
        try:
            with open(eval_path) as f:
                eval_data = json.load(f)
            if "ate_sim3" in eval_data:
                ate = eval_data["ate_sim3"]
                result["status"] = "OK"
                result["ate_rmse"] = ate.get("rmse")
                result["ate_mean"] = ate.get("mean")
                result["ate_median"] = ate.get("median")
                result["ate_std"] = ate.get("std")
                result["ate_max"] = ate.get("max")
                result["num_estimated"] = eval_data.get("num_estimated")
                result["num_gt"] = eval_data.get("num_gt")
                result["tracking_rate_pct"] = eval_data.get("tracking_rate_pct")
                result["sim3_scale"] = eval_data.get("sim3_scale")
                return result
            else:
                # eval_results.json exists but no ate_sim3
                result["fail_reason"] = "eval incomplete"
                if "error" in eval_data:
                    result["fail_reason"] = eval_data["error"]
        except (json.JSONDecodeError, KeyError) as e:
            result["fail_reason"] = f"eval parse error: {e}"

    # no valid eval results -- diagnose from log
    if result["fail_reason"] is None:
        result["fail_reason"] = diagnose_log(log_path)
    else:
        # We already have a fail reason from eval, but augment with log info
        log_diag = diagnose_log(log_path)
        if log_diag not in ("no log", "empty log", "unknown"):
            result["fail_reason"] += f" + {log_diag}"

    return result


def format_cell(entry: dict) -> str:
    """Format a table cell for one recording+mode"""
    if entry["status"] == "OK":
        return f"{entry['ate_rmse']:.4f}m"
    else:
        reason = entry.get("fail_reason", "unknown")
        return f"FAIL ({reason})"


def main():
    print("=" * 70)
    print("ROVER ORB-SLAM3 Summary Regeneration")
    print("=" * 70)
    print()

    # =========================================================================
    # 1. Process all recordings x modes
    # =========================================================================
    all_results = []
    # keyed by (recording, mode)
    result_map = {}

    for rec in RECORDINGS:
        for mode in MODES:
            entry = process_entry(rec, mode)
            all_results.append(entry)
            result_map[(rec, mode)] = entry
            status_str = (
                f"{entry['ate_rmse']:.4f}m"
                if entry["status"] == "OK"
                else f"FAIL ({entry['fail_reason']})"
            )
            print(f"  {rec:40s} | {mode:17s} | {status_str}")

    print()

    # =========================================================================
    # 2. Write summary.txt
    # =========================================================================
    summary_path = BASE_DIR / "summary.txt"

    # column widths
    rec_width = max(len(r) for r in RECORDINGS)
    rec_width = max(rec_width, len("Recording"))
    stereo_width = 34
    si_width = 34
    rgbd_width = 24

    lines = []
    lines.append("ROVER ORB-SLAM3 Baseline Results (Experiment 1.1)")
    lines.append("=" * 49)
    lines.append("")
    lines.append("Generated by regenerate_summary.py")
    lines.append(f"Base directory: {BASE_DIR}")
    lines.append("")
    lines.append(
        "Note: Canonical recording names are used. Some on-disk directory names differ:"
    )
    for canonical, actual in sorted(DIR_MAPPING.items()):
        lines.append(f"  {canonical}  ->  {actual}")
    lines.append("")

    # --- Main results table ---
    hdr = (
        f"{'Recording':<{rec_width}} | "
        f"{'Stereo':<{stereo_width}} | "
        f"{'Stereo-Inertial':<{si_width}} | "
        f"{'RGB-D (ATE RMSE)':<{rgbd_width}}"
    )
    sep = (
        f"{'-' * rec_width}-+-"
        f"{'-' * stereo_width}-+-"
        f"{'-' * si_width}-+-"
        f"{'-' * rgbd_width}"
    )
    lines.append(hdr)
    lines.append(sep)

    for rec in RECORDINGS:
        stereo_cell = format_cell(result_map[(rec, "stereo")])
        si_cell = format_cell(result_map[(rec, "stereo_inertial")])
        rgbd_cell = format_cell(result_map[(rec, "rgbd")])
        lines.append(
            f"{rec:<{rec_width}} | "
            f"{stereo_cell:<{stereo_width}} | "
            f"{si_cell:<{si_width}} | "
            f"{rgbd_cell:<{rgbd_width}}"
        )

    lines.append("")
    lines.append("")

    # =========================================================================
    # 3. Compute statistics
    # =========================================================================
    lines.append("Statistics")
    lines.append("=" * 50)
    lines.append("")

    # --- Per-mode stats ---
    lines.append("Per-Mode Summary:")
    lines.append("-" * 50)
    mode_labels = {
        "stereo": "Stereo",
        "stereo_inertial": "Stereo-Inertial",
        "rgbd": "RGB-D",
    }

    for mode in MODES:
        mode_entries = [result_map[(r, mode)] for r in RECORDINGS]
        successes = [e for e in mode_entries if e["status"] == "OK"]
        failures = [e for e in mode_entries if e["status"] != "OK"]
        n_total = len(mode_entries)
        n_ok = len(successes)
        rate = 100.0 * n_ok / n_total if n_total > 0 else 0

        lines.append("")
        lines.append(f"  {mode_labels[mode]}:")
        lines.append(f"    Success rate: {n_ok}/{n_total} ({rate:.1f}%)")

        if successes:
            ates = [e["ate_rmse"] for e in successes]
            lines.append(f"    ATE RMSE  - mean:   {statistics.mean(ates):.4f}m")
            lines.append(f"    ATE RMSE  - median: {statistics.median(ates):.4f}m")
            lines.append(f"    ATE RMSE  - min:    {min(ates):.4f}m")
            lines.append(f"    ATE RMSE  - max:    {max(ates):.4f}m")
            if len(ates) > 1:
                lines.append(f"    ATE RMSE  - std:    {statistics.stdev(ates):.4f}m")
        else:
            lines.append("    (no successful runs)")

        if failures:
            lines.append("    Failure reasons:")
            reason_counts = {}
            for e in failures:
                r = e.get("fail_reason", "unknown")
                reason_counts[r] = reason_counts.get(r, 0) + 1
            for reason, count in sorted(reason_counts.items(), key=lambda x: -x[1]):
                lines.append(f"      - {reason}: {count}")

    lines.append("")
    lines.append("")

    # --- Per-location stats for RGB-D ---
    lines.append("Per-Location RGB-D Stats:")
    lines.append("-" * 50)

    for location, prefix in [("garden_large", "garden_large_"), ("park", "park_")]:
        loc_recs = [r for r in RECORDINGS if r.startswith(prefix)]
        loc_entries = [result_map[(r, "rgbd")] for r in loc_recs]
        successes = [e for e in loc_entries if e["status"] == "OK"]
        n_total = len(loc_entries)
        n_ok = len(successes)
        rate = 100.0 * n_ok / n_total if n_total > 0 else 0

        lines.append("")
        lines.append(f"  {location} ({n_ok}/{n_total} succeeded, {rate:.1f}%):")
        if successes:
            ates = [e["ate_rmse"] for e in successes]
            lines.append(f"    ATE RMSE  - mean:   {statistics.mean(ates):.4f}m")
            lines.append(f"    ATE RMSE  - median: {statistics.median(ates):.4f}m")
            lines.append(f"    ATE RMSE  - min:    {min(ates):.4f}m")
            lines.append(f"    ATE RMSE  - max:    {max(ates):.4f}m")
            # list individual results
            lines.append("    Individual results:")
            for e in successes:
                lines.append(f"      {e['recording']:<42s} {e['ate_rmse']:.4f}m")
        else:
            lines.append("    (no successful runs)")

        failures = [e for e in loc_entries if e["status"] != "OK"]
        if failures:
            lines.append("    Failed recordings:")
            for e in failures:
                lines.append(
                    f"      {e['recording']:<42s} {e.get('fail_reason', 'unknown')}"
                )

    lines.append("")
    lines.append("")

    # --- Best/worst recordings (across RGB-D, since stereo/SI all failed) ---
    lines.append("Best/Worst Recordings (RGB-D, by ATE RMSE):")
    lines.append("-" * 50)

    rgbd_successes = [
        result_map[(r, "rgbd")]
        for r in RECORDINGS
        if result_map[(r, "rgbd")]["status"] == "OK"
    ]
    if rgbd_successes:
        sorted_by_ate = sorted(rgbd_successes, key=lambda e: e["ate_rmse"])

        lines.append("")
        lines.append("  Best (lowest ATE RMSE):")
        for e in sorted_by_ate[:3]:
            lines.append(f"    {e['recording']:<42s} {e['ate_rmse']:.4f}m")

        lines.append("")
        lines.append("  Worst (highest ATE RMSE):")
        for e in sorted_by_ate[-3:][::-1]:
            lines.append(f"    {e['recording']:<42s} {e['ate_rmse']:.4f}m")

    lines.append("")
    lines.append("")

    # --- Overall summary ---
    lines.append("Overall Summary:")
    lines.append("-" * 50)
    total = len(all_results)
    total_ok = sum(1 for e in all_results if e["status"] == "OK")
    lines.append(f"  Total experiments: {total} (15 recordings x 3 modes)")
    lines.append(
        f"  Total successes:   {total_ok}/{total} ({100.0 * total_ok / total:.1f}%)"
    )
    lines.append(f"  Total failures:    {total - total_ok}/{total}")
    lines.append("")
    lines.append("  Key findings:")

    # compute per-mode stats for the key findings
    for mode in MODES:
        mode_entries = [result_map[(r, mode)] for r in RECORDINGS]
        successes = [e for e in mode_entries if e["status"] == "OK"]
        n_total = len(mode_entries)
        n_ok = len(successes)

        if n_ok == 0:
            # summarize failure reasons
            reasons = set(e.get("fail_reason", "unknown") for e in mode_entries)
            reason_str = ", ".join(sorted(reasons))
            lines.append(
                f"    - {mode_labels[mode]}: 0/{n_total} success "
                f"({reason_str})"
            )
        else:
            ates = [e["ate_rmse"] for e in successes]
            lines.append(
                f"    - {mode_labels[mode]}: {n_ok}/{n_total} succeeded, "
                f"mean ATE RMSE = {statistics.mean(ates):.4f}m, "
                f"median = {statistics.median(ates):.4f}m"
            )

    lines.append("")

    summary_text = "\n".join(lines) + "\n"

    with open(summary_path, "w") as f:
        f.write(summary_text)
    print(f"Written summary to: {summary_path}")
    print()

    # =========================================================================
    # 4. Write all_results.json
    # =========================================================================
    all_results_path = BASE_DIR / "all_results.json"

    # build a clean JSON-serializable list
    json_results = []
    for entry in all_results:
        obj = {
            "recording": entry["recording"],
            "mode": entry["mode"],
            "status": entry["status"],
        }
        if entry["status"] == "OK":
            obj["num_estimated"] = entry["num_estimated"]
            obj["num_gt"] = entry["num_gt"]
            obj["tracking_rate_pct"] = entry["tracking_rate_pct"]
            obj["sim3_scale"] = entry["sim3_scale"]
            obj["ate_sim3"] = {
                "rmse": entry["ate_rmse"],
                "mean": entry["ate_mean"],
                "median": entry["ate_median"],
                "std": entry["ate_std"],
                "max": entry["ate_max"],
            }
        else:
            obj["fail_reason"] = entry["fail_reason"]

        json_results.append(obj)

    with open(all_results_path, "w") as f:
        json.dump(json_results, f, indent=2)
    print(f"Written all_results.json to: {all_results_path}")
    print()

    # =========================================================================
    # 5. Print the summary to stdout
    # =========================================================================
    print(summary_text)


if __name__ == "__main__":
    main()
