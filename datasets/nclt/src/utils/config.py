"""NCLT SLAM Pipeline config"""
import os

NCLT_DATA_ROOT = '/workspace/nclt_data'
VELODYNE_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'velodyne_data')
SENSOR_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'sensor_data')
GROUND_TRUTH_DIR = os.path.join(NCLT_DATA_ROOT, 'ground_truth')
HOKUYO_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'hokuyo_data')
IMAGES_DIR = os.path.join(NCLT_DATA_ROOT, 'images')

# 4 of 27 NCLT sessions, one per season. Cameras only for spring and summer.
SESSIONS = ['2012-01-08', '2012-04-29', '2012-08-04', '2012-10-28']
SESSIONS_WITH_IMAGES = ['2012-04-29', '2012-08-04']

# Velodyne HDL-32E raw .bin encoding: each coord is uint16, value * 0.005 - 100.0
# gives meters (5 mm step, -100..+100 m range).
VELODYNE_SCALING = 0.005
VELODYNE_OFFSET = -100.0
VELODYNE_NUM_LASERS = 32

# Ladybug3: 5 side cameras + 1 sky-facing top (Cam0, unusable for SLAM).
LADYBUG_NUM_CAMERAS = 6

VELODYNE_SYNC_PATTERN = '{session}/velodyne_sync/*.bin'
GROUND_TRUTH_FILE = 'groundtruth_{session}.csv'
