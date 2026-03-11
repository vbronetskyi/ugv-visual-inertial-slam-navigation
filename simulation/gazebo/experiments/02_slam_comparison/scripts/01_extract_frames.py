#!/usr/bin/env python3
"""
Extract RGB, Depth, IMU from rosbag into TUM-style format for ORB-SLAM3
Usage: ros2 bag play bags/route_1_clean --rate 2.0
       python3 01_extract_frames.py  (in another terminal)

Output: orb_slam3_data/{rgb/, depth/, rgb.txt, depth.txt, imu.txt, associations.txt}
"""
import rclpy, numpy as np, cv2, time, os
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu

OUT = '/workspace/simulation/orb_slam3_data'
os.makedirs(f'{OUT}/rgb', exist_ok=True)
os.makedirs(f'{OUT}/depth', exist_ok=True)

def img_to_np(msg):
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    elif msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    return None

class Extractor(Node):
    def __init__(self):
        super().__init__('extractor')
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.dep_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 100)
        self.rf = open(f'{OUT}/rgb.txt', 'w')
        self.df = open(f'{OUT}/depth.txt', 'w')
        self.imf = open(f'{OUT}/imu.txt', 'w')
        self.af = open(f'{OUT}/associations.txt', 'w')
        self.rc = self.dc = self.ic = 0
        self.last_dt = None; self.last_dfn = None
        self.t0 = time.time()

    def rgb_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        img = img_to_np(msg)
        if img is None: return
        fn = f'rgb/{t:.6f}.png'
        cv2.imwrite(f'{OUT}/{fn}', img)
        self.rf.write(f'{t:.6f} {fn}\n')
        if self.last_dt:
            self.af.write(f'{t:.6f} {fn} {self.last_dt:.6f} {self.last_dfn}\n')
        self.rc += 1
        if self.rc % 500 == 0:
            print(f'  RGB:{self.rc} D:{self.dc} IMU:{self.ic} [{time.time()-self.t0:.0f}s]', flush=True)

    def dep_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d = img_to_np(msg)
        if d is None: return
        if d.dtype == np.float32: d = (d * 1000).astype(np.uint16)
        fn = f'depth/{t:.6f}.png'
        cv2.imwrite(f'{OUT}/{fn}', d)
        self.df.write(f'{t:.6f} {fn}\n')
        self.last_dt = t; self.last_dfn = fn
        self.dc += 1

    def imu_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g = msg.angular_velocity; a = msg.linear_acceleration
        self.imf.write(f'{t:.6f} {g.x:.6f} {g.y:.6f} {g.z:.6f} {a.x:.6f} {a.y:.6f} {a.z:.6f}\n')
        self.ic += 1

    def close(self):
        self.rf.close(); self.df.close(); self.imf.close(); self.af.close()
        print(f'\nDone: RGB={self.rc} D={self.dc} IMU={self.ic}', flush=True)

rclpy.init()
n = Extractor()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
n.close()
n.destroy_node()
rclpy.shutdown()
