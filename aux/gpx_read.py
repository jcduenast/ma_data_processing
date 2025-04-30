import pandas as pd
import glob
import gpxpy
import os
from matplotlib import pyplot as plt
import numpy as np


def get_df_from_gpx(gpx_path):
    # Load the file
    with open(gpx_path, 'r') as f:
        gpx = gpxpy.parse(f)

    points = []

    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                points.append({
                    'latitude': point.latitude,
                    'longitude': point.longitude,
                    'elevation': point.elevation,
                    'time': point.time
                })

    df = pd.DataFrame(points)
    return df


def plot_gps_from_df (gps_df):
    plt.figure(figsize=(10, 6))
    plt.plot(gps_df['longitude'], gps_df['latitude'], marker='o', linestyle='-', color='blue')
    plt.title("GPS Track from GPX File")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.axis('equal')  # Preserve aspect ratio
    plt.show()


def plot_speed_from_gps_df(gps_df):
    pos_diff = []
    for i in range(gps_df.shape[0]-1):
        d_lat = gps_df['latitude'][i] - gps_df['latitude'][i+1]
        d_lon = gps_df['longitude'][i] - gps_df['longitude'][i+1]
        pos_diff.append(np.sqrt(d_lat**2+d_lon**2))

    plt.figure(figsize=(10, 6))
    plt.plot(pos_diff) #, marker='o', linestyle='-', color='blue')
    plt.title("GPS 'Speed' ")
    plt.xlabel("step")
    plt.ylabel("quasi-speed sq")
    plt.show()

def plot_gps_altitude_from_gps_df(gps_df):
    plt.figure(figsize=(10, 6))
    plt.plot(gps_df['elevation']) #, marker='o', linestyle='-', color='blue')
    plt.title("GPS Altitude MSL")
    plt.xlabel("step")
    plt.ylabel("Altitude [m]")
    plt.show()


if(__name__) == "__main__":
    folder_path = 'data/80m_1'
    gpx_files = glob.glob(os.path.join(folder_path, '*.gpx'))
    if not gpx_files:
        raise FileNotFoundError("No .gpx file found in the specified folder.")
    gpx_file_path = gpx_files[0]

    df = get_df_from_gpx(gpx_file_path)
    # df = df[['latitude', 'longitude']]
    print("Data size", df.shape)

    plot_gps_from_df(df)
    plot_speed_from_gps_df(df)
    plot_gps_altitude_from_gps_df(df)

    # df.plot()
    # plt.show()

