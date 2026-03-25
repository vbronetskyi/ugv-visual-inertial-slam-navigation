#!/usr/bin/env python3
"""
Traversability filter for depth images.
Distinguishes solid obstacles (tree trunks, rocks) from traversable
vegetation (leaves, grass, ferns) based on depth variance and coverage.

Vegetation: sparse, varying depth (leaves at different distances).
Solid: dense, uniform depth (flat trunk surface).
"""
import numpy as np


class TraversabilityFilter:
    def __init__(self):
        self.VARIANCE_THRESHOLD = 0.5   # m^2 - high variance = vegetation
        self.COVERAGE_THRESHOLD = 0.3   # <30% valid pixels = sparse = traversable
        self.MIN_SOLID_DEPTH = 0.8      # vegetation closer than this = still obstacle
        self.BLOCK_H = 40
        self.BLOCK_W = 40
        self.MAX_RANGE = 10.0

    def filter(self, depth_image):
        if depth_image is None:
            return depth_image

        h, w = depth_image.shape[:2]
        filtered = depth_image.copy()

        for by in range(h // self.BLOCK_H):
            for bx in range(w // self.BLOCK_W):
                y1 = by * self.BLOCK_H
                y2 = y1 + self.BLOCK_H
                x1 = bx * self.BLOCK_W
                x2 = x1 + self.BLOCK_W

                block = depth_image[y1:y2, x1:x2]
                valid = block[
                    (block > 0.3) & (block < self.MAX_RANGE) & np.isfinite(block)
                ]

                if len(valid) < 10:
                    continue

                coverage = len(valid) / (self.BLOCK_H * self.BLOCK_W)
                variance = float(np.var(valid))
                min_depth = float(np.min(valid))

                # Traversable: high variance + low coverage + not too close
                if (variance > self.VARIANCE_THRESHOLD
                        and coverage < self.COVERAGE_THRESHOLD
                        and min_depth > self.MIN_SOLID_DEPTH):
                    filtered[y1:y2, x1:x2] = self.MAX_RANGE

        return filtered

    def stats(self, depth_raw, depth_filtered):
        if depth_raw is None or depth_filtered is None:
            return ""
        raw_blocked = np.sum(
            (depth_raw > 0.3) & (depth_raw < 2.5) & np.isfinite(depth_raw)
        )
        filt_blocked = np.sum(
            (depth_filtered > 0.3) & (depth_filtered < 2.5) & np.isfinite(depth_filtered)
        )
        total = depth_raw.size
        return (f"raw_blk={100*raw_blocked/total:.0f}% "
                f"filt_blk={100*filt_blocked/total:.0f}%")
