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
                'data': msg  # You'll likely want to unpack the message here
            })

    return pd.DataFrame(messages)


db3_reader = get_db3_reader(folder_path)

# ## Search for the topic and msg needed
# topics_types = db3_reader.get_all_topics_and_types()
# print("Topics in the bag:")
# for topic in topics_types:
#     print(f"{topic.name}: {topic.type}")

# dataframe
lidar_df = get_topic_df_from_db3_reader('/bf/points_raw', 'sensor_msgs/msg/PointCloud2', db3_reader)
print(lidar_df.head())



lidar_frame = lidar_df['data'][4000]
print(type(lidar_frame))

points = list(point_cloud2.read_points(lidar_frame, field_names=("x", "y", "z"), skip_nans=True))
print("points in list", len(points))
print(points[0])


print("This file has:", lidar_df.shape[0], "frames.")

datetime_start = datetime.datetime.fromtimestamp(lidar_df['timestamp'][0] / 1e9)
datetime_end = datetime.datetime.fromtimestamp(lidar_df['timestamp'][lidar_df.shape[0]-1] / 1e9)

print(datetime_start, "to", datetime_end )


