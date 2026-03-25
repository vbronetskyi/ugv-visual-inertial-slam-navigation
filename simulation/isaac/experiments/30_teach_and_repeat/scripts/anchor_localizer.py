#!/usr/bin/env python3
"""
Route-relative localizer: determines current position on a taught route
by visual matching against anchor keyframes.
"""
import cv2
import numpy as np
import json
import os
import time


class AnchorLocalizer:
    def __init__(self, anchor_dir, window_back=5, window_forward=15,
                 direction=None):
        self.anchor_dir = anchor_dir
        self.window_back = window_back
        self.window_forward = window_forward

        with open(f"{anchor_dir}/anchors.json") as f:
            all_anchors = json.load(f)

        if direction is not None:
            self.anchors = [a for a in all_anchors if a["direction"] == direction]
            for i, a in enumerate(self.anchors):
                a["_orig_id"] = a["id"]
                a["id"] = i
        else:
            self.anchors = all_anchors

        self.descriptors = {}
        self.keypoints = {}
        for anchor in self.anchors:
            oid = anchor.get("_orig_id", anchor["id"])
            desc_path = f"{anchor_dir}/rgb/{oid:04d}_desc.npy"
            kp_path = f"{anchor_dir}/rgb/{oid:04d}_kp.npy"
            if os.path.exists(desc_path):
                self.descriptors[anchor["id"]] = np.load(desc_path)
                self.keypoints[anchor["id"]] = np.load(kp_path)

        dir_str = f" direction={direction}" if direction else ""
        print(f"AnchorLocalizer: {len(self.anchors)} anchors, "
              f"{len(self.descriptors)} descriptors{dir_str}")

        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.current_anchor_id = 0
        self.confidence = 0.0
        self.match_history = []
        self.HISTORY_SIZE = 5

        self.MIN_GOOD_MATCHES = 15
        self.GOOD_MATCH_RATIO = 0.75
        self.CONFIDENCE_THRESHOLD = 0.3

    def localize(self, current_rgb_frame):
        t0 = time.time()

        if len(current_rgb_frame.shape) == 3:
            gray = cv2.cvtColor(current_rgb_frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = current_rgb_frame

        kp_cur, desc_cur = self.orb.detectAndCompute(gray, None)

        if desc_cur is None or len(kp_cur) < 10:
            return self.current_anchor_id, 0.0, self.anchors[self.current_anchor_id]

        search_start = max(0, self.current_anchor_id - self.window_back)
        search_end = min(len(self.anchors),
                         self.current_anchor_id + self.window_forward)

        best_id = self.current_anchor_id
        best_score = 0
        scores = {}

        for aid in range(search_start, search_end):
            if aid not in self.descriptors:
                continue

            try:
                matches = self.matcher.knnMatch(desc_cur,
                                                self.descriptors[aid], k=2)
            except cv2.error:
                continue

            good = 0
            for pair in matches:
                if len(pair) == 2 and pair[0].distance < self.GOOD_MATCH_RATIO * pair[1].distance:
                    good += 1

            scores[aid] = good
            if good > best_score:
                best_score = good
                best_id = aid

        confidence = min(1.0, best_score / max(self.MIN_GOOD_MATCHES * 2, 1))

        if confidence > self.CONFIDENCE_THRESHOLD:
            max_fwd = 1
            max_back = 2
            delta = best_id - self.current_anchor_id
            if delta > max_fwd:
                best_id = self.current_anchor_id + max_fwd
                confidence *= 0.7
            elif delta < -max_back:
                best_id = self.current_anchor_id - max_back
                confidence *= 0.5

            self.current_anchor_id = best_id
            self.confidence = confidence

        self.match_history.append({
            "anchor_id": best_id,
            "confidence": confidence,
            "score": best_score,
            "time": time.time(),
        })
        if len(self.match_history) > self.HISTORY_SIZE:
            self.match_history.pop(0)

        elapsed_ms = (time.time() - t0) * 1000
        return self.current_anchor_id, self.confidence, self.anchors[self.current_anchor_id]

    def get_lookahead_anchor(self, lookahead_distance=10.0):
        current_s = self.anchors[self.current_anchor_id]["s"]
        target_s = current_s + lookahead_distance
        for anchor in self.anchors[self.current_anchor_id:]:
            if anchor["s"] >= target_s:
                return anchor
        return self.anchors[-1]

    def get_progress(self):
        current_s = self.anchors[self.current_anchor_id]["s"]
        total_s = self.anchors[-1]["s"]
        return (current_s / total_s) * 100.0 if total_s > 0 else 0.0

    def get_stats(self):
        a = self.anchors[self.current_anchor_id]
        return {
            "anchor_id": self.current_anchor_id,
            "confidence": self.confidence,
            "progress": f"{self.get_progress():.1f}%",
            "s": a["s"],
            "x": a["x"],
            "y": a["y"],
        }
