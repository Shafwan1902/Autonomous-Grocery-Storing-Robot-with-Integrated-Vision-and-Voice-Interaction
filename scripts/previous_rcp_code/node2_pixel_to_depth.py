#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point

class DepthExtractor:
    def __init__(self):
        rospy.init_node("depth_extractor", anonymous=True)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.depth_callback)
        rospy.Subscriber("/detected_object_bbox", Int32MultiArray, self.pixel_callback)
        self.depth_points = None
        self.point_pub = rospy.Publisher("/object_position", Point, queue_size=10)
        rospy.loginfo("✅ Depth extractor initialized.")

    def depth_callback(self, msg):
        self.depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))

    def pixel_callback(self, msg):
        if self.depth_points is None:
            rospy.logwarn("Depth data not available yet.")
            return

        x, y = msg.data
        width = 640
        index = y * width + x
        if index >= len(self.depth_points):
            rospy.logwarn("Pixel index out of bounds")
            return

        u, v, depth = self.depth_points[index]
        if any(map(lambda d: d != d, (u, v, depth))):  # NaN check
            rospy.logwarn("❌ Invalid depth value at pixel (%d, %d)", x, y)
            return

        # Camera intrinsics (from /camera/color/camera_info)
        fx = 514.0127563476562
        fy = 514.0127563476562
        cx = 325.4051513671875
        cy = 245.53640747070312

        # Convert pixel + depth to 3D coordinates
        X = (x - cx) * depth / fx
        Y = (y - cy) * depth / fy
        Z = depth

        # Publish the 3D point
        point = Point(X, Y, Z)
        self.point_pub.publish(point)

        # Show in terminal
        rospy.loginfo("✅ 3D position of pixel (%d, %d): X=%.3f, Y=%.3f, Z=%.3f", x, y, X, Y, Z)

if __name__ == "__main__":
    DepthExtractor()
    rospy.spin()
