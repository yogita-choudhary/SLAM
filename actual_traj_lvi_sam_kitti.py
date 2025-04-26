import rosbag
from nav_msgs.msg import Path

# Path to your .bag file
bag_path = "lvi_sam_kitti_0027_results.bag"
topic_name = "/lvi_sam/lidar/mapping/path"
output_file = "pose_output_lvi_sam.txt"

with rosbag.Bag(bag_path, 'r') as bag, open(output_file, 'w') as f:
    for _, msg, _ in bag.read_messages(topics=[topic_name]):
        if hasattr(msg, 'poses'):
            for pose_stamped in msg.poses:
                pos = pose_stamped.pose.position
                ori = pose_stamped.pose.orientation
                
                # Get the timestamp from header.stamp
                timestamp = pose_stamped.header.stamp.secs + pose_stamped.header.stamp.nsecs * 1e-9
                
                # Write data in evo-compatible format
                f.write(f"{timestamp} {pos.x} {pos.y} {pos.z} {ori.w} {ori.x} {ori.y} {ori.z}\n")
