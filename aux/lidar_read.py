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

from aux.rosbag_aux import get_db3_reader

def get_lidar_df_from_db3_reader(rosbag_reader):
    msg_type = rosidl_runtime_py.utilities.get_message('sensor_msgs/msg/PointCloud2')
    rosbag_reader.seek(0)
    messages = []

    while rosbag_reader.has_next():
        (topic_name, data, t) = rosbag_reader.read_next()
        if topic_name == '/bf/points_raw':
            msg = deserialize_message(data, msg_type)
            messages.append({
                'timestamp': t,
                'timestamp_sample': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'np_points': point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True),
                'points_len': msg.width,
                'data': msg
            })

    return pd.DataFrame(messages)


def filter_out_points(np_frame):
    return np_frame


def get_lidar_sample(lidar_df, sample_num):
    return lidar_df['np_points'][sample_num]


def plot_lidar_sample(np_array):##
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(np_array[:,0], np_array[:,1], np_array[:,2], marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()


def get_altitude_from_lidar_pc(points_array, center_tuple=(0,0), square_width=5):
    altitude = []
    for arr in points_array:
        mean = 0
        count = 0
        for p in arr:
            # depth is in y coordinate, x forwards positiv, z left positiv
            if center_tuple[0]-square_width < p[0] and p[0] < center_tuple[0]+square_width:
                if center_tuple[1]-square_width < p[2] and p[2] < center_tuple[1]+square_width:
                    mean += p[1]
                    count += 1
        if count != 0:
            altitude.append(mean/count)
        else:
            altitude.append(0)
    return altitude


if(__name__) == "__main__":
    folder_path = 'data/80m_1'
    frame_num = 4000 # the frame in time to be looked at
    db3_reader = get_db3_reader(folder_path, "ROSBAG*")
    lidar_df = get_lidar_df_from_db3_reader(db3_reader)
    plot_lidar_sample(get_lidar_sample(lidar_df, 1000))