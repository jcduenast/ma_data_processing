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

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2

import datetime


folder_path = 'data/6m_1'

def get_db3_reader(base_path):
    db3_files = glob.glob(os.path.join(base_path, 'ROSBAG*', '*.db3'))
    if not db3_files:
        raise FileNotFoundError("No .db3 file found under any ROSBAG* folder.")
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


def filter_out_points(np_frame):
    return np_frame


def get_single_ros_msg_from_topic_df(topic_df, msg_num):
    return topic_df['data'][msg_num]


def get_lidar_np_frame_from_msg(msg):
    return point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)


def plot_np_3d_points(np_array):##
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(np_array[:,0], np_array[:,1], np_array[:,2], marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

frame_num = 4000 # the frame in time to be looked at
db3_reader = get_db3_reader(folder_path)
lidar_df = get_topic_df_from_db3_reader('/bf/points_raw', 'sensor_msgs/msg/PointCloud2', db3_reader)
print("This file has:", lidar_df.shape[0], "frames. Consulting frame ", frame_num)
lidar_msg = get_single_ros_msg_from_topic_df(lidar_df, frame_num)
np_array = get_lidar_np_frame_from_msg(lidar_msg)
# print("---------------------------------------------px4 message:", type(lidar_msg.header), lidar_msg.header)
print("Frame timestamp   : ", lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec / 1e9)
print("Message time stamp: ", lidar_df['timestamp'][frame_num])
print("Amount of points in msg", np_array.shape[0])
plot_np_3d_points(np_array)

# datetime_start = datetime.datetime.fromtimestamp(lidar_df['timestamp'][0] / 1e9)
# datetime_end = datetime.datetime.fromtimestamp(lidar_df['timestamp'][lidar_df.shape[0]-1] / 1e9)

# print(datetime_start, "to", datetime_end )


# ## Search for the topic and msg needed
# topics_types = db3_reader.get_all_topics_and_types()
# print("Topics in the bag:")
# for topic in topics_types:
#     print(f"{topic.name}: {topic.type}")

# /absolute_pose: geometry_msgs/msg/Pose
# /bf/points_raw: sensor_msgs/msg/PointCloud2
# /drone_acceleration: geometry_msgs/msg/Vector3Stamped
# /drone_angular_rate: geometry_msgs/msg/Vector3Stamped
# /drone_attitude: geometry_msgs/msg/QuaternionStamped
# /drone_fused_height: std_msgs/msg/Float32
# /drone_velocity: geometry_msgs/msg/Vector3Stamped
# /estimator/radar_pointcloud: sensor_msgs/msg/PointCloud2
# /fix: sensor_msgs/msg/NavSatFix
# /gps_heading: geometry_msgs/msg/Vector3Stamped
# /imu/calibration_status/accelerometer: std_msgs/msg/UInt8
# /imu/calibration_status/gyroscope: std_msgs/msg/UInt8
# /imu/calibration_status/magnetometer: std_msgs/msg/UInt8
# /imu/imu: sensor_msgs/msg/Imu
# /imu/mag: sensor_msgs/msg/MagneticField
# /srr308/altitude: std_msgs/msg/Float32
# /srr308/altitude_point: sensor_msgs/msg/PointCloud2
# /srr308/pointcloud: sensor_msgs/msg/PointCloud2
# /tf: tf2_msgs/msg/TFMessage
# /vel: geometry_msgs/msg/TwistStamped