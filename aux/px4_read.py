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

from px4_msgs.msg import SensorGyro
from px4_msgs.msg import SensorCombined, SensorAccel

import datetime


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


def get_gyro_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('px4_msgs/msg/SensorGyro')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/fmu/out/sensor_gyro':
            msg = deserialize_message(data, msg_type)
            t_sample = msg.timestamp_sample
            x = msg.x
            y = msg.y
            z = msg.z
            temperature = msg.temperature
            messages.append({
                'timestamp': t,
                'timestamp_sample': t_sample / 1e6,
                'x': x,
                'y': y,
                'z': z,
                'temperature': temperature
            })
    return pd.DataFrame(messages)


def plot_gyro_from_df(gyro_df):
    plt.figure(figsize=(10, 6))
    plt.plot(gyro_df['timestamp_sample'], gyro_df['x'], color='red')
    plt.plot(gyro_df['timestamp_sample'], gyro_df['y'], color='green')
    plt.plot(gyro_df['timestamp_sample'], gyro_df['z'], color='blue')
    plt.title("Gyroscope")
    plt.xlabel("time")
    plt.ylabel("rad sec")
    # plt.show()


def get_acc_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('px4_msgs/msg/SensorAccel')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/fmu/out/sensor_accel':
            msg = deserialize_message(data, msg_type)
            t_sample = msg.timestamp_sample
            x = msg.x
            y = msg.y
            z = msg.z
            temperature = msg.temperature
            messages.append({
                'timestamp': t,
                'timestamp_sample': t_sample / 1e6,
                'x': x,
                'y': y,
                'z': z,
                'temperature': temperature
            })
    return pd.DataFrame(messages)


def plot_acc_from_df(acc_df):
    plt.figure(figsize=(10, 6))
    plt.plot(acc_df['timestamp_sample'], acc_df['x'], color='red')
    plt.plot(acc_df['timestamp_sample'], acc_df['y'], color='green')
    plt.plot(acc_df['timestamp_sample'], acc_df['z'], color='blue')
    plt.title("Accelerometer")
    plt.xlabel("time")
    plt.ylabel("m sec^2")
    # plt.show()


def get_sensor_combined_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('px4_msgs/msg/SensorCombined')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/fmu/out/sensor_combined':
            msg = deserialize_message(data, msg_type)
            gyro_arr = msg.gyro_rad
            acc_arr = msg.accelerometer_m_s2
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.timestamp / 1e6,
                'gyro_x': gyro_arr[0],
                'gyro_y': gyro_arr[1],
                'gyro_z': gyro_arr[2],
                'acc_x': acc_arr[0],
                'acc_y': acc_arr[1],
                'acc_z': acc_arr[2]
            })
    return pd.DataFrame(messages)

def plot_sensor_combined_from_df(sc_df):
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


def get_baro_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('px4_msgs/msg/SensorBaro')
    rosbag_reader.seek(0)
    messages = []
    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/fmu/out/sensor_baro':
            msg = deserialize_message(data, msg_type)
            t_sample = msg.timestamp_sample
            pressure = msg.pressure
            temperature = msg.temperature
            messages.append({
                'timestamp': t,
                'timestamp_sample': t_sample / 1e6,
                'pressure': pressure,
                'temperature': temperature
            })
    return pd.DataFrame(messages)


def plot_baro_from_df(baro_df):
    plt.figure(figsize=(10, 6))
    plt.plot(baro_df['timestamp_sample'], baro_df['pressure'], color='blue')
    plt.title("Pressure")
    plt.xlabel("time")
    plt.ylabel("Pa")
    # plt.show()


if(__name__) == "__main__":
    folder_path = 'data/50m_2'

    df_entry_num = 0
    db3_reader = get_db3_reader(folder_path, 'sensor_bag*')
    px4_df = get_topic_df_from_db3_reader('/fmu/out/sensor_baro', 'px4_msgs/msg/SensorBaro', db3_reader)

    print("Dataframe shape", px4_df.shape)
    msg = px4_df['data'][df_entry_num]
    print("px4 message:", type(msg), msg)

    print("Sample timestamp : ", msg.timestamp)
    # print("Frame timestamp   : ", single_msg.header.stamp.sec + single_msg.header.stamp.nanosec / 1e9)
    print("Arrival timestamp: ", px4_df['timestamp'][df_entry_num])

    baro_df = get_baro_df_from_db3_reader(db3_reader)
    print("first pressure at", folder_path, ":", baro_df['pressure'][0])
    # plot_baro_from_df(baro_df)

    gyro_df = get_gyro_df_from_db3_reader(db3_reader)
    plot_gyro_from_df(gyro_df)

    acc_df = get_acc_df_from_db3_reader(db3_reader)
    plot_acc_from_df(acc_df)

    sc_df = get_sensor_combined_df_from_db3_reader(db3_reader)
    plot_sensor_combined_from_df(sc_df)

    plt.show()


