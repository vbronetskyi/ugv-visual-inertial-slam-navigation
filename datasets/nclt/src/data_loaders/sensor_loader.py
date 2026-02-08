"""
Sensor data loader for NCLT (IMU, GPS, Odometry)

Sensors: MS25 IMU, GPS, GPS RTK, KVH FOG, wheel odometry
Ref: https://robots.engin.umich.edu/nclt/
"""
import os
import pandas as pd
import numpy as np
from typing import Dict, Optional


class SensorLoader:
    """loader for NCLT sensor data (IMU, GPS, odometry)"""

    def __init__(self, data_root: str = '/workspace/nclt_data/sensor_data'):
        self.data_root = data_root

    def load_ms25_imu(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'ms25.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"MS25 IMU file not found: {filepath}")

        df = pd.read_csv(filepath, header=None, names=[
            'utime', 'mag_x', 'mag_y', 'mag_z',
            'accel_x', 'accel_y', 'accel_z',
            'rot_x', 'rot_y', 'rot_z'
        ])
        return df

    def load_ms25_euler(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'ms25_euler.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"MS25 Euler file not found: {filepath}")

        df = pd.read_csv(filepath, header=None, names=[
            'utime', 'roll', 'pitch', 'yaw'
        ])
        return df

    def load_gps(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'gps.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"GPS file not found: {filepath}")

        # no header, many columns
        df = pd.read_csv(filepath, header=None)
        return df

    def load_gps_rtk(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'gps_rtk.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"GPS RTK file not found: {filepath}")

        df = pd.read_csv(filepath, header=None)
        return df

    def load_odometry_mu(self, session: str, hz: int = 10) -> pd.DataFrame:
        """load integrated odometry poses, hz=10 or 100"""
        if hz == 100:
            filename = 'odometry_mu_100hz.csv'
        else:
            filename = 'odometry_mu.csv'

        filepath = os.path.join(self.data_root, session, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Odometry file not found: {filepath}")

        df = pd.read_csv(filepath, header=None, names=[
            'utime', 'x', 'y', 'z',
            'roll', 'pitch', 'yaw'
        ])
        return df

    def load_odometry_cov(self, session: str, hz: int = 10) -> pd.DataFrame:
        if hz == 100:
            filename = 'odometry_cov_100hz.csv'
        else:
            filename = 'odometry_cov.csv'

        filepath = os.path.join(self.data_root, session, filename)
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Odometry covariance file not found: {filepath}")

        df = pd.read_csv(filepath, header=None)
        return df

    def load_kvh(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'kvh.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"KVH file not found: {filepath}")

        df = pd.read_csv(filepath, header=None, names=['utime', 'heading'])
        return df

    def load_wheels(self, session: str) -> pd.DataFrame:
        filepath = os.path.join(self.data_root, session, 'wheels.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Wheels file not found: {filepath}")

        df = pd.read_csv(filepath, header=None, names=['utime', 'left_speed', 'right_speed'])
        return df

    def load_all_sensors(self, session: str) -> Dict[str, pd.DataFrame]:
        sensors = {}

        try:
            sensors['ms25_imu'] = self.load_ms25_imu(session)
        except FileNotFoundError:
            pass

        try:
            sensors['ms25_euler'] = self.load_ms25_euler(session)
        except FileNotFoundError:
            pass

        try:
            sensors['gps'] = self.load_gps(session)
        except FileNotFoundError:
            pass

        try:
            sensors['gps_rtk'] = self.load_gps_rtk(session)
        except FileNotFoundError:
            pass

        try:
            sensors['odometry_mu'] = self.load_odometry_mu(session)
        except FileNotFoundError:
            pass

        try:
            sensors['odometry_mu_100hz'] = self.load_odometry_mu(session, hz=100)
        except FileNotFoundError:
            pass

        try:
            sensors['odometry_cov'] = self.load_odometry_cov(session)
        except FileNotFoundError:
            pass

        try:
            sensors['kvh'] = self.load_kvh(session)
        except FileNotFoundError:
            pass

        try:
            sensors['wheels'] = self.load_wheels(session)
        except FileNotFoundError:
            pass

        return sensors
