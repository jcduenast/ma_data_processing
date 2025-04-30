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
from sensor_msgs.msg import PointCloud2, NavSatFix, MagneticField
from geometry_msgs.msg import Vector3Stamped, TwistStamped, Vector3
from tf2_msgs.msg import TFMessage

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


def get_navsat_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/NavSatFix')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/fix':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'position_covariance': msg.position_covariance,
            })
    return pd.DataFrame(messages)


def plot_navsat_data_from_df(navsat_df, show=False):
    plt.figure(figsize=(10, 6))
    plt.plot(navsat_df['longitude'], navsat_df['latitude'], marker='o', linestyle='-', color='blue')
    plt.title("Navsat")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.axis('equal')

    plt.figure(figsize=(10, 6))
    plt.plot(navsat_df['timestamp_sample'], navsat_df['altitude'], marker='o', linestyle='-', color='blue')
    plt.title("Navsat")
    plt.xlabel("Timestamp")
    plt.ylabel("Altitude [m]")

    time_diff = [navsat_df['timestamp_sample'][i+1]-navsat_df['timestamp_sample'][i] for i in range(navsat_df.shape[0]-1)]
    print(navsat_df['timestamp_sample'][30], time_diff[30])
    plt.figure(figsize=(10, 6))
    plt.plot(time_diff, color='blue')
    plt.title("Timestamp difference")
    plt.xlabel("Datapoint")
    plt.ylabel("Delta [s]")

    if show:
        plt.show()


def get_navsat_heading_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('geometry_msgs/msg/Vector3Stamped')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/gps_heading':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'x': msg.vector.x,
                'y': msg.vector.y,
                'z': msg.vector.z,
            })
    return pd.DataFrame(messages)


def plot_navsat_heading_data_from_df(heading_df, show=False):
    plt.figure(figsize=(10, 6))
    plt.plot(heading_df['timestamp_sample'], heading_df['z'], marker='o', linestyle='-', color='blue')
    plt.title("Navsat heading")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Heading [rad] ? ")    
    if show:
        plt.show()


def get_velocity_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('geometry_msgs/msg/TwistStamped')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/vel':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'vel_x': msg.twist.linear.x,
                'vel_y': msg.twist.linear.y,
                'vel_z': msg.twist.linear.z,
                'ang_x': msg.twist.angular.x,
                'ang_y': msg.twist.angular.y,
                'ang_z': msg.twist.angular.z
            })
    return pd.DataFrame(messages)


def plot_velocity_from_df(vel_df, show=False):
    plt.figure(figsize=(10, 6))
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_x'], color='red')
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_y'], color='green')
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_z'], color='blue')
    plt.title("Linear velocity")
    plt.xlabel("time")
    plt.ylabel("velocity [m/s]")
    # plt.show()

    # plt.figure(figsize=(10, 6))
    # plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='red')
    # plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='green')
    # plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='blue')
    # plt.title("Angular velocity")
    # plt.xlabel("time")
    # plt.ylabel("Angular velocity [rad/s]")
    
    if show:
        plt.show()
    

def get_speed_from_navsat_velocity_df(vel_df):
    speed = []
    for i in range(vel_df.shape[0]):
        _speed = np.sqrt(vel_df['vel_x'][i]**2+vel_df['vel_y'][i]**2+vel_df['vel_z'][i]**2)
        speed.append(_speed)
    return np.array(speed)

def plot_speed_from_np(timestamps, speed_array):
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, speed_array, color='red')
    plt.title("Linear velocity")
    plt.xlabel("time [s]")
    plt.ylabel("velocity [m/s]")


if(__name__) == "__main__":
    folder_path = 'data/80m_1'
    df_entry_num = 30
    db3_reader = get_db3_reader(folder_path, 'ROSBAG*')

    # navsat_df = get_navsat_df_from_db3_reader(db3_reader)
    # plot_navsat_data_from_df(navsat_df)

    # heading_df = get_navsat_heading_df_from_db3_reader(db3_reader)
    # plot_navsat_heading_data_from_df(heading_df)

    vel_df = get_velocity_df_from_db3_reader(db3_reader)
    plot_velocity_from_df(vel_df)

    speed_array = get_speed_from_navsat_velocity_df(vel_df)
    plot_speed_from_np(vel_df['timestamp_sample'], speed_array)
    plt.show()






