import os
import numpy as np
import pandas as pd
from pyproj import Transformer
from scipy.spatial.transform import Rotation as R

# Paths (update as needed)
timestamps_path = '/path/to/oxts/timestamps.txt'
data_folder = '/path/to/oxts/data'

# Load timestamps
with open(timestamps_path, 'r') as f:
    timestamps = [line.strip() for line in f.readlines()]

# Sorted list of data files
data_files = sorted([os.path.join(data_folder, f) for f in os.listdir(data_folder) if f.endswith('.txt')])

# Coordinate transformer (adjust EPSG zone if needed)
transformer = Transformer.from_crs("epsg:4326", "epsg:32632", always_xy=True)

trajectory = []
origin_easting = origin_northing = origin_alt = None

for ts, file_path in zip(timestamps, data_files):
    with open(file_path, 'r') as f:
        values = list(map(float, f.read().strip().split()))
        lat, lon, alt = values[0], values[1], values[2]
        roll, pitch, yaw = values[3], values[4], values[5]

    easting, northing = transformer.transform(lon, lat)

    if origin_easting is None:
        origin_easting, origin_northing, origin_alt = easting, northing, alt

    x = easting - origin_easting
    y = northing - origin_northing
    z = alt - origin_alt

    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()  # [x, y, z, w]

    trajectory.append([ts, x, y, z, *quat])

# Write manually to avoid trailing spaces
output_file = 'oxts_trajectory.txt'
with open(output_file, 'w') as f:
    for row in trajectory:
        f.write(' '.join(map(str, row)) + '\n')
