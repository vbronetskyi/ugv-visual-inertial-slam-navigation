"""
Hokuyo 2D LiDAR loader for NCLT

Binary: 8 bytes timestamp + 1081 x 2 bytes range (uint16)
Range: actual = raw * 0.005 - 100.0

Ref: https://github.com/aljosaosep/NCLT-dataset-tools
"""
import os
import struct
import numpy as np
from typing import List, Tuple


class HokuyoLoader:
    """loader for Hokuyo UTM-30LX-EW 2D LiDAR"""

    SCALING = 0.005  # 5mm per unit
    OFFSET = -100.0
    NUM_RANGES_30M = 1081  # Number of range measurements for 30m Hokuyo
    NUM_RANGES_4M = 726    # Number of range measurements for 4m Hokuyo

    def __init__(self, data_root: str = '/workspace/nclt_data/hokuyo_data'):
        self.data_root = data_root

    @staticmethod
    def convert_range(raw_value: int) -> float:
        return raw_value * HokuyoLoader.SCALING + HokuyoLoader.OFFSET

    def load_hokuyo_30m(self, session: str) -> List[dict]:
        filepath = os.path.join(self.data_root, session, 'hokuyo_30m.bin')
        return self._load_hokuyo_file(filepath, self.NUM_RANGES_30M)

    def load_hokuyo_4m(self, session: str) -> List[dict]:
        filepath = os.path.join(self.data_root, session, 'hokuyo_4m.bin')
        return self._load_hokuyo_file(filepath, self.NUM_RANGES_4M)

    def _load_hokuyo_file(self, filepath: str, num_ranges: int) -> List[dict]:
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Hokuyo file not found: {filepath}")

        scans = []
        bytes_per_scan = 8 + (num_ranges * 2)  # ts + ranges

        with open(filepath, 'rb') as f:
            while True:
                # timestamp
                utime_bytes = f.read(8)
                if not utime_bytes:  # EOF
                    break

                utime = struct.unpack('<Q', utime_bytes)[0]

                # range measurements
                ranges = np.zeros(num_ranges, dtype=np.float32)
                for i in range(num_ranges):
                    raw_range = struct.unpack('<H', f.read(2))[0]
                    ranges[i] = self.convert_range(raw_range)

                scans.append({
                    'utime': utime,
                    'ranges': ranges
                })

        return scans

    def ranges_to_cartesian(self, ranges: np.ndarray,
                           angle_min: float = -2.35619,  # -135 degrees
                           angle_max: float = 2.35619,   # 135 degrees
                           filter_invalid: bool = True) -> np.ndarray:
        """polar ranges to Nx2 [x, y] cartesian"""
        num_ranges = len(ranges)
        angles = np.linspace(angle_min, angle_max, num_ranges)

        # polar to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        points = np.column_stack([x, y])

        if filter_invalid:
            # filter invalid (negative values)
            valid_mask = ranges > 0
            points = points[valid_mask]

        return points

    def get_scan_statistics(self, scans: List[dict]) -> dict:
        if not scans:
            return {}

        all_ranges = np.concatenate([scan['ranges'] for scan in scans])
        valid_ranges = all_ranges[all_ranges > 0]

        return {
            'num_scans': len(scans),
            'ranges_per_scan': len(scans[0]['ranges']),
            'duration_s': (scans[-1]['utime'] - scans[0]['utime']) / 1e6,
            'min_range': valid_ranges.min() if len(valid_ranges) > 0 else 0,
            'max_range': valid_ranges.max() if len(valid_ranges) > 0 else 0,
            'mean_range': valid_ranges.mean() if len(valid_ranges) > 0 else 0,
        }
