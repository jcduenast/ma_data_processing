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


folder_path = 'data/80m_1'

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

def plot_imu_from_df(sc_df):
    plt.figure(figsize=(10, 6))
    plt.plot(sc_df['timestamp_sample'], sc_df['q_x'], color='red')
    plt.plot(sc_df['timestamp_sample'], sc_df['q_y'], color='green')
    plt.plot(sc_df['timestamp_sample'], sc_df['q_z'], color='blue')
    plt.plot(sc_df['timestamp_sample'], sc_df['q_w'], color='black')
    plt.title("Orientation")
    plt.xlabel("time")
    # plt.ylabel("")
    # plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(sc_df['timestamp_sample'], sc_df['gyro_x'], color='red')
    plt.plot(sc_df['timestamp_sample'], sc_df['gyro_y'], color='green')
    plt.plot(sc_df['timestamp_sample'], sc_df['gyro_z'], color='blue')
    plt.title("Gyroscope")
    plt.xlabel("time")
    plt.ylabel("rad sec")
    # plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(sc_df['timestamp_sample'], sc_df['acc_x'], color='red')
    plt.plot(sc_df['timestamp_sample'], sc_df['acc_y'], color='green')
    plt.plot(sc_df['timestamp_sample'], sc_df['acc_z'], color='blue')
    plt.title("Accelerometer")
    plt.xlabel("time")
    plt.ylabel("m sec^2")
    # plt.show()




df_entry_num = 0
db3_reader = get_db3_reader(folder_path, 'sensor_bag*')


# 
# db3_reader = get_db3_reader(folder_path, 'ROSBAG*')
# px4_df = get_topic_df_from_db3_reader('/imu/mag', 'sensor_msgs/msg/MagneticField', db3_reader)


print("Dataframe shape", px4_df.shape)
msg = px4_df['data'][df_entry_num]
print("px4 message:", type(msg), msg)
print("imu message:", type(msg), msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
print("timestamp sample : ", msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)

# SensorCombined.tim
# SensorAccel
# Imu.header.

# sensor_msgs.msg.Imu(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1745834178, nanosec=311961203), frame_id='imu'), 
#                     orientation=geometry_msgs.msg.Quaternion(x=-0.450927734375, y=-0.89251708984375, z=0.00384521484375, w=0.006591796875), 
#                     orientation_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]), 
#                     angular_velocity=geometry_msgs.msg.Vector3(x=0.0, y=-0.002181661564992912, z=-0.001090830782496456), 
#                     angular_velocity_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.]), 
#                     linear_acceleration=geometry_msgs.msg.Vector3(x=0.08, y=-0.22, z=-9.75), 
#                     linear_acceleration_covariance=array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
#                     )

# print("Sample timestamp : ", msg.timestamp)
# print("Frame timestamp   : ", single_msg.header.stamp.sec + single_msg.header.stamp.nanosec / 1e9)
print("Arrival timestamp: ", px4_df['timestamp'][df_entry_num])





imu_df = get_imu_df_from_db3_reader(db3_reader)
plot_imu_from_df(imu_df)

plt.show()


# lidar_msg = get_single_ros_msg_from_topic_df(lidar_df, 0)
# np_array = get_lidar_np_frame_from_msg(lidar_msg)
# print("Amount of points in msg", np_array.shape[0])
# plot_np_3d_points(np_array)

# print("This file has:", lidar_df.shape[0], "frames.")

# datetime_start = datetime.datetime.fromtimestamp(lidar_df['timestamp'][0] / 1e9)
# datetime_end = datetime.datetime.fromtimestamp(lidar_df['timestamp'][lidar_df.shape[0]-1] / 1e9)

# print(datetime_start, "to", datetime_end )


# # Search for the topic and msg needed
# topics_types = db3_reader.get_all_topics_and_types()
# print("Topics in the bag:")
# for topic in topics_types:
#     print(f"{topic.name}: {topic.type}")

