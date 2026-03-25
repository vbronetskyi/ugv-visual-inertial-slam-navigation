#!/usr/bin/env python3
"""
Robust anchor localizer with sequence matching, depth verification,
and conservative correction. Replaces anchor_localizer.py for exp 32+.
"""
import os
import json
import math
import numpy as np
import cv2


class RobustAnchorLocalizer:
    def __init__(self, anchor_dir, direction=None):
        self.anchor_dir = anchor_dir

        with open(f"{anchor_dir}/anchors.json") as f:
            all_anchors = json.load(f)

        if direction is not None:
            self.anchors = [a for a in all_anchors if a["direction"] == direction]
            for i, a in enumerate(self.anchors):
                a["_orig_id"] = a["id"]
                a["id"] = i
        else:
            self.anchors = all_anchors

        # Load ORB descriptors
        self.descriptors = {}
        for anchor in self.anchors:
            oid = anchor.get("_orig_id", anchor["id"])
            desc_path = f"{anchor_dir}/rgb/{oid:04d}_desc.npy"
            if os.path.exists(desc_path):
                self.descriptors[anchor["id"]] = np.load(desc_path)

        # Load depth profiles (if available)
        self.anchor_depths = {}
        for anchor in self.anchors:
            oid = anchor.get("_orig_id", anchor["id"])
            dp_path = f"{anchor_dir}/rgb/{oid:04d}_depth_profile.npy"
            if os.path.exists(dp_path):
                self.anchor_depths[anchor["id"]] = np.load(dp_path)

        dir_str = f" direction={direction}" if direction else ""
        print(f"RobustAnchorLocalizer: {len(self.anchors)} anchors, "
              f"{len(self.descriptors)} desc, "
              f"{len(self.anchor_depths)} depth profiles{dir_str}")

        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # State
        self.current_anchor_id = 0
        self.confirmed_anchor_id = 0
        self.confidence = 0.0

        # Sequence matching
        self.SEQUENCE_LENGTH = 5
        self.SEQUENCE_AGREEMENT = 3
        self.frame_votes = []

        # Conservative correction
        self.pending_anchor_id = None
        self.pending_count = 0
        self.CONFIRM_FRAMES = 3
        self.MAX_CORRECTION = 3

        # Matching params
        self.GOOD_RATIO = 0.75
        self.WINDOW_BACK = 3
        self.WINDOW_FWD = 10

    def localize(self, rgb_frame, depth_frame=None):
        raw_id, raw_conf = self._orb_match(rgb_frame)

        if depth_frame is not None and raw_id in self.anchor_depths:
            depth_score = self._depth_match(depth_frame, raw_id)
            combined = 0.6 * raw_conf + 0.4 * depth_score
        else:
            combined = raw_conf

        self.frame_votes.append((raw_id, combined))
        if len(self.frame_votes) > self.SEQUENCE_LENGTH:
            self.frame_votes.pop(0)

        if len(self.frame_votes) >= self.SEQUENCE_AGREEMENT:
            cons_id, cons_conf = self._get_consensus()
        else:
            cons_id, cons_conf = raw_id, combined

        final_id = self._conservative_update(cons_id, cons_conf)

        self.current_anchor_id = final_id
        self.confidence = cons_conf

        return final_id, cons_conf, self.anchors[final_id]

    def _orb_match(self, rgb_frame):
        if len(rgb_frame.shape) == 3:
            gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)
        else:
            gray = rgb_frame
        gray_eq = self.clahe.apply(gray)
        kp, desc = self.orb.detectAndCompute(gray_eq, None)

        if desc is None or len(kp) < 10:
            return self.confirmed_anchor_id, 0.0

        center = self.confirmed_anchor_id
        s0 = max(0, center - self.WINDOW_BACK)
        s1 = min(len(self.anchors), center + self.WINDOW_FWD)

        best_id, best_score = self.confirmed_anchor_id, 0
        for aid in range(s0, s1):
            if aid not in self.descriptors:
                continue
            try:
                ms = self.matcher.knnMatch(desc, self.descriptors[aid], k=2)
            except cv2.error:
                continue
            good = sum(1 for pair in ms
                       if len(pair) == 2
                       and pair[0].distance < self.GOOD_RATIO * pair[1].distance)
            if good > best_score:
                best_score = good
                best_id = aid

        return best_id, min(1.0, best_score / 30.0)

    def _depth_match(self, depth_frame, candidate_id):
        current_profile = self._depth_to_profile(depth_frame)
        anchor_profile = self.anchor_depths.get(candidate_id)

        if current_profile is None or anchor_profile is None:
            return 0.5

        cn = current_profile - np.mean(current_profile)
        an = anchor_profile - np.mean(anchor_profile)
        denom = np.std(cn) * np.std(an) * len(cn)
        if denom < 0.001:
            return 0.5

        corr = float(np.sum(cn * an) / denom)
        return max(0.0, (corr + 1.0) / 2.0)

    def _depth_to_profile(self, depth_frame, n_bins=32):
        if depth_frame is None:
            return None
        h, w = depth_frame.shape[:2]
        strip = depth_frame[h // 3: h * 2 // 3, :]
        bw = w // n_bins
        profile = np.zeros(n_bins)
        for i in range(n_bins):
            col = strip[:, i * bw: (i + 1) * bw]
            valid = col[(col > 0.3) & (col < 10.0) & np.isfinite(col)]
            profile[i] = float(np.median(valid)) if len(valid) > 3 else 5.0
        return profile

    def _get_consensus(self):
        counts = {}
        for aid, conf in self.frame_votes:
            if aid not in counts:
                counts[aid] = {"n": 0, "tc": 0.0}
            counts[aid]["n"] += 1
            counts[aid]["tc"] += conf

        best_id = self.confirmed_anchor_id
        best_n = 0
        best_tc = 0.0
        for aid, info in counts.items():
            if info["n"] > best_n or (info["n"] == best_n and info["tc"] > best_tc):
                best_id = aid
                best_n = info["n"]
                best_tc = info["tc"]

        avg = best_tc / best_n if best_n > 0 else 0.0
        return best_id, avg

    def _conservative_update(self, consensus_id, confidence):
        if confidence < 0.3:
            return self.confirmed_anchor_id

        diff = consensus_id - self.confirmed_anchor_id
        if abs(diff) > self.MAX_CORRECTION:
            return self.confirmed_anchor_id

        if diff == 0:
            self.pending_anchor_id = None
            self.pending_count = 0
            return self.confirmed_anchor_id

        if self.pending_anchor_id == consensus_id:
            self.pending_count += 1
        else:
            self.pending_anchor_id = consensus_id
            self.pending_count = 1

        if self.pending_count >= self.CONFIRM_FRAMES:
            self.confirmed_anchor_id = consensus_id
            self.pending_anchor_id = None
            self.pending_count = 0
            return self.confirmed_anchor_id

        return self.confirmed_anchor_id

    def get_stats(self):
        a = self.anchors[self.confirmed_anchor_id]
        return {
            "confirmed_id": self.confirmed_anchor_id,
            "current_id": self.current_anchor_id,
            "confidence": self.confidence,
            "pending": f"{self.pending_anchor_id}({self.pending_count})"
                       if self.pending_anchor_id is not None else "none",
            "s": a["s"],
            "x": a["x"],
            "y": a["y"],
        }
