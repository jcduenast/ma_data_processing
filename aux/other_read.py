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


def plot_navsat_data_from_df(navsat_df):
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
    plt.show()


def get_navsat_heading_df_from_db3_reader(rosbag_reader):   ######### edit
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


def plot_navsat_heading_data_from_df(heading_df):   ######### edit
    plt.figure(figsize=(10, 6))
    plt.plot(heading_df['timestamp_sample'], heading_df['z'], marker='o', linestyle='-', color='blue')
    plt.title("Navsat heading")
    plt.xlabel("Timestamp [s]")
    plt.ylabel("Heading [rad] ? ")
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


def plot_velocity_from_df(vel_df):
    plt.figure(figsize=(10, 6))
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_x'], color='red')
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_y'], color='green')
    plt.plot(vel_df['timestamp_sample'], vel_df['vel_z'], color='blue')
    plt.title("Linear velocity")
    plt.xlabel("time")
    plt.ylabel("velocity [m/s]")
    # plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='red')
    plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='green')
    plt.plot(vel_df['timestamp_sample'], vel_df['ang_x'], color='blue')
    plt.title("Angular velocity")
    plt.xlabel("time")
    plt.ylabel("Angular velocity [rad/s]")
    plt.show()


if(__name__) == "__main__":
    folder_path = 'data/80m_1'

    df_entry_num = 30
    db3_reader = get_db3_reader(folder_path, 'ROSBAG*')

    df = get_topic_df_from_db3_reader('/vel', 'geometry_msgs/msg/TwistStamped', db3_reader)
    print("msg", df['data'][70].twist.linear.x, df['data'][70].twist.angular)


    # TFMessage.transforms.

    # time_diff = [navsat_df['timestamp_sample'][i+1]-navsat_df['timestamp_sample'][i] for i in range(navsat_df.shape[0]-1)]
    # print(navsat_df['timestamp_sample'][30], time_diff[30])
    # plt.figure(figsize=(10, 6))
    # plt.plot(time_diff, color='blue')
    # plt.show()

    # df = get_topic_df_from_db3_reader('/drone_acceleration', 'geometry_msgs/msg/Vector3Stamped', db3_reader) # 0
    # df = get_topic_df_from_db3_reader('/drone_angular_rate', 'geometry_msgs/msg/Vector3Stamped', db3_reader) # 0

    # # Search for the topic and msg needed
    # topics_types = db3_reader.get_all_topics_and_types()
    # print("Topics in the bag:")
    # for topic in topics_types:
    #     if topic.type != 'radar_interface/msg/RadarSensorStatus':
    #         df = get_topic_df_from_db3_reader(topic.name, topic.type, db3_reader)
    #         print(topic.name, topic.type, df.shape)

    # Topics in the bag:
    # /tf tf2_msgs/msg/TFMessage (27556, 2) ## maybe compare orientation to the IMU quaternions, seems like are the same
    # /vel geometry_msgs/msg/TwistStamped (1748, 2)


    ### Clean code:

    # navsat_df = get_navsat_df_from_db3_reader(db3_reader)
    # plot_navsat_data_from_df(navsat_df)
    # heading_df = get_navsat_heading_df_from_db3_reader(db3_reader)
    # plot_navsat_heading_data_from_df(heading_df)
    vel_df = get_velocity_df_from_db3_reader(db3_reader)
    plot_velocity_from_df(vel_df)
