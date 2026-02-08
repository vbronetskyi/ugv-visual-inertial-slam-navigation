"""
Velodyne HDL-32E loader for NCLT.

NCLT binary format is quirky: per point it stores x/y/z as uint16 then
intensity + laser_id as uint8 each, so 8 bytes total (the extra byte is
essentially padding). recover metric coords via:

    actual[m] = raw * 0.005 - 100.0

range is therefore [-100 m, +227.675 m] which covers HDL-32E's 100 m range
plus the platform offset. there's no explicit endianness marker but little
endian works (matches the NCLT dataset tools repo linked below).

Ref: https://github.com/aljosaosep/NCLT-dataset-tools
"""
import os
import struct
import numpy as np
from typing import Tuple, Optional
import glob

class VelodyneLoader:
    """velodyne HDL-32E point cloud loader"""

    # 5 mm quantization + 100 m offset -> range [-100, +227.675] m
    SCALING = 0.005
    OFFSET = -100.0
    # NCLT docs say 7 bytes per point, but actual layout is 8 bytes with
    # one byte of alignment padding. we read the layout explicitly below,
    # this constant is just documentation
    BYTES_PER_POINT = 7

    def __init__(self, data_root: str = '/workspace/nclt_data/velodyne_data'):
        self.data_root = data_root

    @staticmethod
    def convert_coordinates(x_raw: int, y_raw: int, z_raw: int) -> Tuple[float, float, float]:
        x = x_raw * VelodyneLoader.SCALING + VelodyneLoader.OFFSET
        y = y_raw * VelodyneLoader.SCALING + VelodyneLoader.OFFSET
        z = z_raw * VelodyneLoader.SCALING + VelodyneLoader.OFFSET
        return x, y, z

    def load_velodyne_sync(self, filepath: str) -> np.ndarray:
        """load single velodyne_sync .bin, returns Nx5 [x,y,z,intensity,laser_id]"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Velodyne file not found: {filepath}")

        points = []

        with open(filepath, 'rb') as f:
            while True:
                # read 8 bytes (x, y, z, intensity, laser_id)
                x_bytes = f.read(2)
                if not x_bytes:  # EOF
                    break

                x_raw = struct.unpack('<H', x_bytes)[0]
                y_raw = struct.unpack('<H', f.read(2))[0]
                z_raw = struct.unpack('<H', f.read(2))[0]
                intensity = struct.unpack('B', f.read(1))[0]
                laser_id = struct.unpack('B', f.read(1))[0]

                # convert to metric coordinates
                x, y, z = self.convert_coordinates(x_raw, y_raw, z_raw)
                points.append([x, y, z, intensity, laser_id])

        return np.array(points, dtype=np.float32)

    def load_velodyne_hits(self, filepath: str, max_packets: Optional[int] = None) -> list:
        """load velodyne_hits.bin, returns list of {utime, hits (Nx5)}"""
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Velodyne hits file not found: {filepath}")

        MAGIC = 44444
        packets = []

        with open(filepath, 'rb') as f:
            packet_count = 0
            while True:
                if max_packets and packet_count >= max_packets:
                    break

                # read magic number (8 bytes = 4 uint16)
                magic_bytes = f.read(8)
                if not magic_bytes:  # EOF
                    break

                magic = struct.unpack('<HHHH', magic_bytes)
                if not all(m == MAGIC for m in magic):
                    print(f"Warning: Invalid magic number at packet {packet_count}")
                    break

                # packet header
                num_hits = struct.unpack('<I', f.read(4))[0]
                utime = struct.unpack('<Q', f.read(8))[0]
                f.read(4)  # padding

                # read hits
                hits = []
                for _ in range(num_hits):
                    x_raw = struct.unpack('<H', f.read(2))[0]
                    y_raw = struct.unpack('<H', f.read(2))[0]
                    z_raw = struct.unpack('<H', f.read(2))[0]
                    intensity = struct.unpack('B', f.read(1))[0]
                    laser_id = struct.unpack('B', f.read(1))[0]

                    x, y, z = self.convert_coordinates(x_raw, y_raw, z_raw)
                    hits.append([x, y, z, intensity, laser_id])

                packets.append({
                    'utime': utime,
                    'hits': np.array(hits, dtype=np.float32)
                })
                packet_count += 1

        return packets

    def get_velodyne_sync_files(self, session: str) -> list:
        sync_dir = os.path.join(self.data_root, session, 'velodyne_sync')
        if not os.path.exists(sync_dir):
            raise FileNotFoundError(f"Velodyne sync directory not found: {sync_dir}")

        files = sorted(glob.glob(os.path.join(sync_dir, '*.bin')))
        return files

    def get_timestamp_from_filename(self, filepath: str) -> int:
        basename = os.path.basename(filepath)
        return int(basename.replace('.bin', ''))
