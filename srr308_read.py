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
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2

from sensor_msgs.msg import Imu


folder_path = 'data/6m_1'

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

def get_srr308_altitude_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/PointCloud2')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/srr308/altitude_point':
            msg = deserialize_message(data, msg_type)
            np_array = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)[0]
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'altitude': np_array[0],
            })
    return pd.DataFrame(messages)


def plot_radar_altimeter_from_df(radar_altitude_df):
    plt.figure(figsize=(10, 6))
    plt.plot(radar_altitude_df['timestamp_sample'], radar_altitude_df['altitude'], color='red')
    plt.title("Altitude")
    plt.xlabel("time")
    plt.ylabel("m")
    plt.show()


def get_srr308_pointcloud_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/PointCloud2')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/srr308/pointcloud':
            msg = deserialize_message(data, msg_type)
            np_array = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'points': np_array,
            })
    return pd.DataFrame(messages)


def plot_radar_points(radar_df, frame_num):
    x = radar_df['points'][frame_num][:,0]
    y = radar_df['points'][frame_num][:,1]
    plt.figure(figsize=(10, 6))
    # np_array = radar_df['points'][frame_num]
    plt.plot(x, y, marker='o', linestyle='-', color='blue')
    plt.title("Radar Point Cloud")
    plt.xlabel("Depth [m]")
    plt.ylabel("Azimuth [m]")
    plt.grid(True)
    plt.axis('equal')  # Preserve aspect ratio
    plt.show()


df_entry_num = 1500
db3_reader = get_db3_reader(folder_path, 'ROSBAG*')
radar_df = get_srr308_pointcloud_df_from_db3_reader(db3_reader)
plot_radar_points(radar_df, df_entry_num)

