## use virtual environment with this code

import geopandas as gpd
import contextily as ctx
import matplotlib.pyplot as plt
from shapely.geometry import LineString
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


def plot_gps_altitude_from_gps_df(gps_df):
    plt.figure(figsize=(10, 6))
    plt.plot(gps_df['elevation']) #, marker='o', linestyle='-', color='blue')
    plt.title("GPS Altitude MSL")
    plt.xlabel("step")
    plt.ylabel("Altitude [m]")
    plt.show()


def plot_navsat_segments_on_map(navsat_df, segments_list, show=True):
    plt.figure(figsize=(14, 7))

    # Create GeoDataFrame for each segment
    for i, (start, end) in enumerate(segments_list):
        segment = navsat_df.iloc[start:end]
        line = LineString(zip(segment['longitude'], segment['latitude']))
        gdf = gpd.GeoDataFrame(index=[0], geometry=[line], crs='EPSG:4326')  # WGS84
        
        # Convert to Web Mercator for contextily
        gdf = gdf.to_crs(epsg=3857)
        gdf.plot(ax=plt.gca(), linewidth=2, label=f"Segment {i}")

    ctx.add_basemap(plt.gca(), crs='EPSG:3857', source=ctx.providers.OpenStreetMap.Mapnik)
    plt.legend()
    plt.title("GPS Tracks Overlaid on Map")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    if show:
        plt.show()


if(__name__) == "__main__":
    folder_path = 'data/80m_1'
    gpx_files = glob.glob(os.path.join(folder_path+'/gpx', '*.gpx'))
    if not gpx_files:
        raise FileNotFoundError("No .gpx file found in the specified folder.")
    gpx_file_path = gpx_files[0]

    df = get_df_from_gpx(gpx_file_path)
    # df = df[['latitude', 'longitude']]
    print("Data size", df.shape)

    plot_gps_from_df(df)
    plot_gps_altitude_from_gps_df(df)

    # df.plot()
    # plt.show()
