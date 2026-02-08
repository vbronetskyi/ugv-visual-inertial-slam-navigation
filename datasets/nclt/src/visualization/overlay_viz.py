"""
Point cloud overlay on camera images for NCLT

Projects Velodyne points onto Ladybug3 camera images
Note: 2012-01-08 has no images, use 2012-04-29 or 2012-08-04
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys
import os
import glob

# add src directory to pathsys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from data_loaders.velodyne_loader import VelodyneLoader
from calibration.calibration import NCLTCalibration


class OverlayVisualizer:
    """Visualize Velodyne points projected onto camera images"""

    def __init__(self, calib_dir: str = None):
        self.velodyne_loader = VelodyneLoader()
        self.calib = NCLTCalibration(calib_dir)

    def load_image(self, session: str, cam_id: int, img_idx: int = 0) -> np.ndarray:
        """Load Ladybug3 camera image as BGR"""
        img_dir = f'/workspace/nclt_data/images/{session}/lb3/Cam{cam_id}'
        if not os.path.exists(img_dir):
            raise FileNotFoundError(f"Image directory not found: {img_dir}")

        img_files = sorted(glob.glob(os.path.join(img_dir, '*.tiff')))
        if not img_files:
            raise FileNotFoundError(f"No images found in {img_dir}")

        if img_idx >= len(img_files):
            img_idx = 0

        img = cv2.imread(img_files[img_idx])
        return img

    def overlay_points_on_image(self, image: np.ndarray, points: np.ndarray,
                                cam_id: int, colors: np.ndarray = None,
                                point_size: int = 3) -> np.ndarray:
        """Project 3D points onto image, returns image with overlaid points"""
        # project points to image
        pixels, valid_mask = self.calib.project_velodyne_to_camera(
            points, cam_id, in_body_frame=False
        )

        # filter points within image bounds
        h, w = image.shape[:2]
        in_bounds = (
            (pixels[:, 0] >= 0) & (pixels[:, 0] < w) &
            (pixels[:, 1] >= 0) & (pixels[:, 1] < h) &
            valid_mask
        )

        valid_pixels = pixels[in_bounds].astype(int)

        # compute colors if not provided
        if colors is None:
            # color by depth (distance from camera)
            depths = points[in_bounds, 2]
            colors = self._depth_colormap(depths)
        else:
            colors = colors[in_bounds]

        # draw points on image
        overlay = image.copy()
        for (u, v), color in zip(valid_pixels, colors):
            color_bgr = tuple(int(c * 255) for c in color[::-1])  # RGB to BGR
            cv2.circle(overlay, (u, v), point_size, color_bgr, -1)

        return overlay

    def visualize_velodyne_overlay(self, session: str, cam_id: int = 0,
                                   scan_idx: int = 0, img_idx: int = 0):
        """Show velodyne points overlaid on camera image (needs session with images)"""
        print(f"Loading data for session {session}, camera {cam_id}")

        # load image
        try:
            image = self.load_image(session, cam_id, img_idx)
            print(f"Loaded image: {image.shape}")
        except FileNotFoundError as e:
            print(f"Error: {e}")
            print("Note: Images only available for sessions 2012-04-29 and 2012-08-04")
            return

        # load Velodyne scan
        vel_files = self.velodyne_loader.get_velodyne_sync_files(session)
        if not vel_files:
            print(f"No Velodyne files found for {session}")
            return

        if scan_idx >= len(vel_files):
            scan_idx = 0

        points_full = self.velodyne_loader.load_velodyne_sync(vel_files[scan_idx])
        points = points_full[:, :3]  # Extract x, y, z
        print(f"Loaded Velodyne scan: {len(points)} points")

        print(f"Projecting points to camera {cam_id}...")
        overlay = self.overlay_points_on_image(image, points, cam_id)

        # display
        plt.figure(figsize=(12, 8))
        plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        plt.title(f"Velodyne Overlay - Session {session}, Camera {cam_id}")
        plt.axis('off')
        plt.tight_layout()
        plt.show()

    @staticmethod
    def _depth_colormap(depths: np.ndarray) -> np.ndarray:
        """depth values to Nx3 RGB colors (0-1)"""
        # normalize depths
        d_min, d_max = depths.min(), depths.max()
        if d_max > d_min:
            normalized = (depths - d_min) / (d_max - d_min)
        else:
            normalized = np.zeros_like(depths)

        # jet colormap
        colors = np.zeros((len(depths), 3))
        for i, v in enumerate(normalized):
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

    parser = argparse.ArgumentParser(
        description='Overlay Velodyne points on Ladybug3 images'
    )
    parser.add_argument('--session', default='2012-04-29',
                       help='Session name (must be 2012-04-29 or 2012-08-04)')
    parser.add_argument('--cam-id', type=int, default=0,
                       help='Camera ID (0-5)')
    parser.add_argument('--scan-idx', type=int, default=0,
                       help='Velodyne scan index')
    parser.add_argument('--img-idx', type=int, default=0,
                       help='Image index')
    parser.add_argument('--calib-dir', default=None,
                       help='Calibration directory (optional)')

    args = parser.parse_args()

    # verify session has images
    if args.session not in ['2012-04-29', '2012-08-04']:
        print(f"Warning: Session {args.session} may not have images.")
        print("Images are only available for 2012-04-29 and 2012-08-04")

    viz = OverlayVisualizer(calib_dir=args.calib_dir)
    viz.visualize_velodyne_overlay(
        session=args.session,
        cam_id=args.cam_id,
        scan_idx=args.scan_idx,
        img_idx=args.img_idx
    )
