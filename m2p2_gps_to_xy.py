import rosbag
import rospy
import math
from sensor_msgs.msg import NavSatFix

# === CONFIGURATION ===
BAG_FILE = "m2p2.bag"  # <-- Change to your rosbag path
TOPIC_NAME = "/sensor_suite/f9p_rover/fix"
OUTPUT_FILE = "output_poses.txt"

# === CONSTANTS ===
EARTH_RADIUS = 6378137.0  # meters (WGS84)

# === FUNCTIONS ===
def latlon_to_meters(lat_origin, lon_origin, lat, lon):
    dlat = math.radians(lat - lat_origin)
    dlon = math.radians(lon - lon_origin)
    lat_origin_rad = math.radians(lat_origin)

    x = EARTH_RADIUS * dlon * math.cos(lat_origin_rad)
    y = EARTH_RADIUS * dlat
    return x, y

# === MAIN ===
if __name__ == "__main__":
    bag = rosbag.Bag(BAG_FILE)

    origin = None
    poses = []

    for topic, msg, t in bag.read_messages(topics=[TOPIC_NAME]):
        # no type check anymore!

        try:
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
        except AttributeError:
            continue  # skip if somehow bad message

        if origin is None:
            origin = (lat, lon, alt)

        x, y = latlon_to_meters(origin[0], origin[1], lat, lon)
        z = alt - origin[2]

        timestamp = t.to_sec()

        # Identity quaternion (no rotation)
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        poses.append(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}")

    bag.close()

    with open(OUTPUT_FILE, "w") as f:
        for pose in poses:
            f.write(pose + "\n")

    print(f"Saved {len(poses)} poses to {OUTPUT_FILE}")
