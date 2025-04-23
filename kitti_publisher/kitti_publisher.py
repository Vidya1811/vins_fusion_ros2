#!/usr/bin/env python3
import os
import rclpy
import cv2
import csv
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Vector3
from builtin_interfaces.msg import Time
from glob import glob
from datetime import datetime

class KITTIPublisher(Node):
    def __init__(self, base_path):
        super().__init__('kitti_publisher')
        self.bridge = CvBridge()

        # Topics
        self.cam0_pub = self.create_publisher(Image, '/cam0/image_raw', 10)
        self.cam1_pub = self.create_publisher(Image, '/cam1/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu0', 50)

        # Load data
        self.cam0_imgs = sorted(glob(os.path.join(base_path, 'image_00/data/*.png')))
        self.cam1_imgs = sorted(glob(os.path.join(base_path, 'image_01/data/*.png')))
        self.oxts_file = os.path.join(base_path, 'oxts/data')
        self.imu_data = self.load_oxts(self.oxts_file)

        self.index = 0
        self.timer = self.create_timer(0.05, self.publish_next)

    def load_oxts(self, oxts_dir):
        imu_list = []
        for file in sorted(glob(os.path.join(oxts_dir, '*.txt'))):
            with open(file, 'r') as f:
                parts = list(map(float, f.readline().split()))
                # parts[17:20] = angular rate (roll/pitch/yaw rate), parts[11:14] = acceleration
                imu_list.append({
                    'accel': parts[11:14],
                    'gyro': parts[17:20]
                })
        return imu_list

    def get_time(self, i):
        # Simulated timestamp
        t = Time()
        t.sec = i // 10
        t.nanosec = (i % 10) * 100000000
        return t

    def publish_next(self):
        if self.index >= len(self.cam0_imgs):
            self.get_logger().info('🔁 Looping playback...')
            self.index = 0
            return

        time_stamp = self.get_time(self.index)

        # Publish cam0
        img0 = cv2.imread(self.cam0_imgs[self.index], cv2.IMREAD_GRAYSCALE)
        msg0 = self.bridge.cv2_to_imgmsg(img0, encoding='mono8')
        msg0.header.stamp = time_stamp
        msg0.header.frame_id = 'cam0'
        self.cam0_pub.publish(msg0)

        # Publish cam1
        img1 = cv2.imread(self.cam1_imgs[self.index], cv2.IMREAD_GRAYSCALE)
        msg1 = self.bridge.cv2_to_imgmsg(img1, encoding='mono8')
        msg1.header.stamp = time_stamp
        msg1.header.frame_id = 'cam1'
        self.cam1_pub.publish(msg1)

        # Publish IMU
        if self.index < len(self.imu_data):
            imu_msg = Imu()
            imu_msg.header.stamp = time_stamp
            imu_msg.header.frame_id = 'imu0'

            ax, ay, az = self.imu_data[self.index]['accel']
            gx, gy, gz = self.imu_data[self.index]['gyro']

            imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
            imu_msg.angular_velocity = Vector3(x=gx, y=gy, z=gz)

            self.imu_pub.publish(imu_msg)

        self.index += 1


def main():
    rclpy.init()
    base_path = os.path.expanduser('~/2011_09_26_drive_0001_sync/2011_09_26/2011_09_26_drive_0001_sync')
    node = KITTIPublisher(base_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

