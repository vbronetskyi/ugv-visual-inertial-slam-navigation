"""
3D point cloud viz for NCLT

Open3D interactive viz of Velodyne and Hokuyo data
"""
import numpy as np
import open3d as o3d
import sys
import os

# add src directory to pathsys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.hokuyo_loader import HokuyoLoader
from data_loaders.ground_truth_loader import GroundTruthLoader


class PointCloudVisualizer:
    """Interactive 3D point cloud visualization"""

    def __init__(self):
        self.velodyne_loader = VelodyneLoader()
        self.hokuyo_loader = HokuyoLoader()
        self.gt_loader = GroundTruthLoader()

    def visualize_velodyne_scan(self, filepath: str,
                                color_by: str = 'height',
                                point_size: float = 2.0):
        """Visualize a single Velodyne scan colored by height/intensity/laser_id/distance"""
        print(f"Loading Velodyne scan: {filepath}")
        points = self.velodyne_loader.load_velodyne_sync(filepath)

        # create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])

        # color points based on chosen attribute
        colors = self._compute_colors(points, color_by)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=[0, 0, 0]
        )

        # visualize
        print(f"Visualizing {len(points)} points (colored by {color_by})")
        print("Controls:")
        print("  - Mouse: Rotate view")
        print("  - Mouse wheel: Zoom")
        print("  - Ctrl + Mouse: Pan")
        print("  - Q or ESC: Exit")

        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name=f"Velodyne Point Cloud - {os.path.basename(filepath)}",
            width=1280,
            height=720,
            point_show_normal=False
        )

    def visualize_hokuyo_scan(self, session: str, scan_idx: int = 0,
                              sensor: str = '30m'):
        """Visualize a Hokuyo 2D scan in 3D"""
        print(f"Loading Hokuyo {sensor} scan {scan_idx} for {session}")

        # load scan
        if sensor == '30m':
            scans = self.hokuyo_loader.load_hokuyo_30m(session)
        else:
            scans = self.hokuyo_loader.load_hokuyo_4m(session)

        if scan_idx >= len(scans):
            print(f"Error: scan_idx {scan_idx} out of range (max: {len(scans)-1})")
            return

        scan = scans[scan_idx]

        # convert to Cartesian
        points_2d = self.hokuyo_loader.ranges_to_cartesian(scan['ranges'])

        # add z=0 to make it 3D
        points_3d = np.column_stack([points_2d, np.zeros(len(points_2d))])

        # create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)
        pcd.paint_uniform_color([1.0, 0.0, 0.0])  # Red

        # create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0, origin=[0, 0, 0]
        )

        # visualize
        print(f"Visualizing {len(points_3d)} points")
        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name=f"Hokuyo {sensor} Scan - {session}",
            width=1280,
            height=720
        )

    def visualize_trajectory(self, session: str, subsample: int = 100):
        """Visualize ground truth trajectory, showing every Nth pose"""
        print(f"Loading ground truth for {session}")
        gt = self.gt_loader.load_ground_truth(session)

        # get positions
        positions = gt[['x', 'y', 'z']].values[::subsample]

        # create line set for trajectory
        points = positions
        lines = [[i, i+1] for i in range(len(points)-1)]

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(points)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.colors = o3d.utility.Vector3dVector(
            [[0, 0, 1] for _ in range(len(lines))]  # Blue
        )

        # create coordinate frames at start and end
        start_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=positions[0]
        )
        end_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=2.0, origin=positions[-1]
        )

        # visualize
        print(f"Visualizing trajectory ({len(positions)} poses)")
        print(f"Start: {positions[0]}")
        print(f"End: {positions[-1]}")

        o3d.visualization.draw_geometries(
            [line_set, start_frame, end_frame],
            window_name=f"Ground Truth Trajectory - {session}",
            width=1280,
            height=720
        )

    def _compute_colors(self, points: np.ndarray, color_by: str) -> np.ndarray:
        """Compute Nx3 RGB colors for point cloud based on chosen attribute"""
        if color_by == 'height':
            values = points[:, 2]  # Z coordinate
        elif color_by == 'intensity':
            values = points[:, 3]
        elif color_by == 'laser_id':
            values = points[:, 4]
        elif color_by == 'distance':
            values = np.linalg.norm(points[:, :3], axis=1)
        else:
            # default: uniform color
            return np.ones((len(points), 3)) * 0.5

        # normalize to 0-1
        v_min, v_max = values.min(), values.max()
        if v_max > v_min:
            normalized = (values - v_min) / (v_max - v_min)
        else:
            normalized = np.zeros_like(values)

        # apply colormap (jet-like)
        colors = self._jet_colormap(normalized)
        return colors

    @staticmethod
    def _jet_colormap(values: np.ndarray) -> np.ndarray:
        """Simple jet colormap implementation"""
        colors = np.zeros((len(values), 3))

        # blue -> Cyan -> Green -> Yellow -> Red
        for i, v in enumerate(values):
            if v < 0.25:
                colors[i] = [0, 4*v, 1]
            elif v < 0.5:
                colors[i] = [0, 1, 1-4*(v-0.25)]
            elif v < 0.75:
                colors[i] = [4*(v-0.5), 1, 0]
            else:
                colors[i] = [1, 1-4*(v-0.75), 0]

        return colors


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Visualize NCLT point clouds')
    parser.add_argument('--session', default='2012-01-08', help='Session name')
    parser.add_argument('--velodyne-idx', type=int, default=0,
                       help='Velodyne scan index to visualize')
    parser.add_argument('--hokuyo-idx', type=int, default=0,
                       help='Hokuyo scan index to visualize')
    parser.add_argument('--color-by', default='height',
                       choices=['height', 'intensity', 'laser_id', 'distance'],
                       help='Color point cloud by attribute')
    parser.add_argument('--mode', default='velodyne',
                       choices=['velodyne', 'hokuyo', 'trajectory'],
                       help='Visualization mode')

    args = parser.parse_args()

    viz = PointCloudVisualizer()

    if args.mode == 'velodyne':
        # get velodyne files
        files = viz.velodyne_loader.get_velodyne_sync_files(args.session)
        if files:
            print(f"Found {len(files)} velodyne scans")
            viz.visualize_velodyne_scan(files[args.velodyne_idx],
                                       color_by=args.color_by)
        else:
            print(f"No velodyne files found for {args.session}")

    elif args.mode == 'hokuyo':
        viz.visualize_hokuyo_scan(args.session, args.hokuyo_idx)

    elif args.mode == 'trajectory':
        viz.visualize_trajectory(args.session)
