from aux.gpx_read import *
from aux.imu_read import *
from aux.lidar_read import *
from aux.navsat_read import *
from aux.px4_read import *
from aux.rosbag_aux import *
from aux.srr308_read import *

import datetime

folder_path = 'data/50m_1'
db3_reader = get_db3_reader(folder_path, 'ROSBAG*')
db3_px4_reader = get_db3_reader(folder_path, 'sensor_bag*')

first_sample = 0
last_sample = 500

radar_df = get_srr308_pointcloud_df_from_db3_reader(db3_reader)
lidar_df = get_lidar_df_from_db3_reader(db3_reader)

first_timestamp = radar_df['timestamp_sample'][first_sample]
print("First timestamp", datetime.datetime.fromtimestamp(first_timestamp).isoformat())

# plot_radar_points(radar_df, df_entry_num)

# radar_sample = plot_radar_points(radar_df, 1000)


lidar_altitude = get_altitude_from_lidar_pc(lidar_df['np_points'], (0,0), 10)
plt.figure()
plt.plot(lidar_df['timestamp_sample'], lidar_altitude)
plt.title("Lidar points centered, 10 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")

lidar_altitude = get_altitude_from_lidar_pc(lidar_df['np_points'], (0,0), 5)
plt.figure()
plt.plot(lidar_df['timestamp_sample'], lidar_altitude)
plt.title("Lidar points centered, 5 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")


lidar_altitude = get_altitude_from_lidar_pc(lidar_df['np_points'], (0,0), 3)
plt.figure()
plt.plot(lidar_df['timestamp_sample'], lidar_altitude)
plt.title("Lidar points centered, 3 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")

##### Show estimation of altitude averaging radar points
radar_altitude = get_altitude_from_radar_pc(radar_df['points'], 0, 10)
plt.figure()
plt.plot(radar_df['timestamp_sample'], radar_altitude)
plt.title("Radar points centered, 10 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")

plt.figure()
radar_altitude = get_altitude_from_radar_pc(radar_df['points'], 0, 5)
plt.plot(radar_df['timestamp_sample'], radar_altitude)
plt.title("Radar points centered, 5 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")

plt.figure()
radar_altitude = get_altitude_from_radar_pc(radar_df['points'], 0, 3)
plt.plot(radar_df['timestamp_sample'], radar_altitude)
plt.title("Radar points centered, 3 m radius")
plt.xlabel("Depth x-axis")
plt.ylabel("Azimuth y-axis")
plt.show()



# navsat_df = get_navsat_df_from_db3_reader(db3_reader)
# plot_navsat_data_from_df(navsat_df)
# heading_df = get_navsat_heading_df_from_db3_reader(db3_reader)
# plot_navsat_heading_data_from_df(heading_df)
# vel_df = get_velocity_df_from_db3_reader(db3_reader)
# plot_velocity_from_df(vel_df)
# plt.show()


