import rosbag
from nav_msgs.msg import Path
import numpy as np
# Path to your .bag file
bag_path = "lvio_sam_m2p2_result.bag"
topic_name = "/lvio_sam/mapping/path"
output_file = "pose_output_lvio_sam_m2p2.txt"

with rosbag.Bag(bag_path, 'r') as bag, open(output_file, 'w') as f:
    for _, msg, _ in bag.read_messages(topics=[topic_name]):
        if hasattr(msg, 'poses'):
            for pose_stamped in msg.poses:
                pos = pose_stamped.pose.position
                ori = pose_stamped.pose.orientation
                
                # Get the timestamp from header.stamp
                timestamp = pose_stamped.header.stamp.secs + pose_stamped.header.stamp.nsecs * 1e-9
                theta = -5*np.pi/6 + 0.3
                c = np.cos(theta)
                s = np.sin(theta)
                Rz = np.array([
                    [ c, -s, 0],
                    [ s,  c, 0],
                    [ 0,  0, 1]])
                pos.x, pos.y, pos.z = np.dot(Rz, np.array([pos.x, pos.y, pos.z]))
                # Write data in evo-compatible format
                f.write(f"{timestamp} {pos.x} {pos.y} {pos.z} {ori.w} {ori.x} {ori.y} {ori.z}\n")


