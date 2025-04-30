import pandas as pd
import glob
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rosidl_runtime_py.utilities
from rclpy.serialization import deserialize_message


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

def get_topic_msgs_df_from_db3_reader(ros_topic, ros_msg_type, rosbag_reader):
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

if(__name__) == "__main__":
    pass