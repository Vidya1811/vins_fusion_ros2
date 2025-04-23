#!/usr/bin/env python3
import os
import sys
import argparse
import csv
import rclpy
import rosbag2_py
from sensor_msgs.msg import Imu, Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime, timedelta
from rclpy.serialization import serialize_message


bridge = CvBridge()

def time_from_ns(ns):
    t = Time()
    t.sec = int(ns / 1e9)
    t.nanosec = int(ns % 1e9)
    return t

def euroc_to_ros2(euroc_path, output_bag_path):
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    writer.create_topic(rosbag2_py.TopicMetadata(name='/imu0', type='sensor_msgs/msg/Imu', serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/cam0/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr'))

    print("Processing IMU...")
    with open(os.path.join(euroc_path, 'mav0', 'imu0', 'data.csv')) as f:
        imu_reader = csv.reader(f)
        next(imu_reader)  # Skip header
        for row in imu_reader:
            t = int(row[0])
            msg = Imu()
            msg.header.stamp = time_from_ns(t)
            msg.linear_acceleration.x = float(row[1])
            msg.linear_acceleration.y = float(row[2])
            msg.linear_acceleration.z = float(row[3])
            msg.angular_velocity.x = float(row[4])
            msg.angular_velocity.y = float(row[5])
            msg.angular_velocity.z = float(row[6])
            msg.header.frame_id = 'imu0'
            writer.write('/imu0', serialize_message(msg), t)

    print("Processing images...")
    with open(os.path.join(euroc_path, 'mav0', 'cam0', 'data.csv')) as f:
        image_reader = csv.reader(f)
        next(image_reader)
        for row in image_reader:
            t = int(row[0])
            img_path = os.path.join(euroc_path, 'mav0', 'cam0', 'data', row[1])
            if not os.path.exists(img_path):
                continue
            cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            msg = bridge.cv2_to_imgmsg(cv_img, encoding='mono8')
            msg.header.stamp = time_from_ns(t)
            msg.header.frame_id = 'cam0'
            writer.write('/cam0/image_raw', serialize_message(msg), t)

    print(f"Done. Bag written to {output_bag_path}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--euroc_path', required=True, help='Path to EuRoC dataset root')
    parser.add_argument('--output_bag', required=True, help='Output .db3 file path')
    args = parser.parse_args()
    euroc_to_ros2(args.euroc_path, args.output_bag)
