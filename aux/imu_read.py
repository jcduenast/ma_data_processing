import pandas as pd
import glob
import gpxpy
import os
from matplotlib import pyplot as plt
import numpy as np
import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd
import rosidl_runtime_py.utilities
from rclpy.serialization import deserialize_message

import datetime

from sensor_msgs.msg import Imu


def get_db3_reader(base_path, folder_clue):
    db3_files = glob.glob(os.path.join(base_path, folder_clue, '*.db3'))
    if not db3_files:
        raise FileNotFoundError("No .db3 file found under any {folder_clue} folder.")
    rosbag_path = db3_files[0]
    print(f"Found ROS 2 bag: {rosbag_path}")
    storage_options = StorageOptions(uri=rosbag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_topic_df_from_db3_reader(ros_topic, ros_msg_type, rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message(ros_msg_type)
    rosbag_reader.seek(0)
    messages = []

    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == ros_topic:
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'data': msg
            })

    return pd.DataFrame(messages)


def get_imu_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/Imu')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/imu/imu':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'q_x': msg.orientation.x,
                'q_y': msg.orientation.y,
                'q_z': msg.orientation.z,
                'q_w': msg.orientation.w,
                'gyro_x': msg.angular_velocity.x,
                'gyro_y': msg.angular_velocity.y,
                'gyro_z': msg.angular_velocity.z,
                'acc_x': msg.linear_acceleration.x,
                'acc_y': msg.linear_acceleration.y,
                'acc_z': msg.linear_acceleration.z
            })
    return pd.DataFrame(messages)


def plot_imu_from_df(imu_df):
    plt.figure(figsize=(10, 6))
    plt.plot(imu_df['timestamp_sample'], imu_df['q_x'], color='red')
    plt.plot(imu_df['timestamp_sample'], imu_df['q_y'], color='green')
    plt.plot(imu_df['timestamp_sample'], imu_df['q_z'], color='blue')
    plt.plot(imu_df['timestamp_sample'], imu_df['q_w'], color='black')
    plt.title("Orientation")
    plt.xlabel("time")
    # plt.ylabel("")
    # plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(imu_df['timestamp_sample'], imu_df['gyro_x'], color='red')
    plt.plot(imu_df['timestamp_sample'], imu_df['gyro_y'], color='green')
    plt.plot(imu_df['timestamp_sample'], imu_df['gyro_z'], color='blue')
    plt.title("Gyroscope")
    plt.xlabel("time")
    plt.ylabel("rad sec")
    # plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(imu_df['timestamp_sample'], imu_df['acc_x'], color='red')
    plt.plot(imu_df['timestamp_sample'], imu_df['acc_y'], color='green')
    plt.plot(imu_df['timestamp_sample'], imu_df['acc_z'], color='blue')
    plt.title("Accelerometer")
    plt.xlabel("time")
    plt.ylabel("m sec^2")
    # plt.show()


def get_mag_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/MagneticField')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/imu/mag':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'mag_x': msg.magnetic_field.x,
                'mag_y': msg.magnetic_field.y,
                'mag_z': msg.magnetic_field.z
            })
    return pd.DataFrame(messages)


def plot_mag_from_df(mag_df):
    plt.figure(figsize=(10, 6))
    plt.plot(mag_df['timestamp_sample'], mag_df['mag_x'], color='red')
    plt.plot(mag_df['timestamp_sample'], mag_df['mag_y'], color='green')
    plt.plot(mag_df['timestamp_sample'], mag_df['mag_z'], color='blue')
    plt.title("Magnetic field vector")
    plt.xlabel("time")
    plt.ylabel("Intensity [G]")
    # plt.show()


if(__name__) == "__main__":
    folder_path = 'data/80m_1'
    df_entry_num = 0
    db3_sensor_reader = get_db3_reader(folder_path, 'ROSBAG*')
    # db3_sensor_reader = get_db3_reader(folder_path, 'sensor_bag*')
    imu_df = get_imu_df_from_db3_reader(db3_sensor_reader)
    plot_imu_from_df(imu_df)

    db3_ROSBAG_reader = get_db3_reader(folder_path, 'ROSBAG*')
    mag_df = get_mag_df_from_db3_reader(db3_ROSBAG_reader)
    plot_mag_from_df(mag_df)
    plt.show()
