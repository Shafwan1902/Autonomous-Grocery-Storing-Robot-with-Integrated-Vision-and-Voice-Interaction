#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from jupiterobot2_msgs.msg import YoloMsg
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CentroidTrackerNode:

    def __init__(self):

        self.bridge = CvBridge()
        self.latest_depth_img = None

        # Subscriber to object detection output
        self.person_sub = rospy.Subscriber(
            'yolo/object_info', 
            YoloMsg, 
            self.detection_callback
        )

        self.depth_sub = rospy.Subscriber(
            "/camera/depth/points",
            PointCloud2,
            self.depth_callback  
        )

        # Publisher for centroids
        self.centroid_pub = rospy.Publisher(
            '/receptionist/human_centroids', 
            PointStamped, 
            queue_size=10
        )

        # Subscriber for depth image (for visualization)
        self.depth_image_sub = rospy.Subscriber(
            "/camera/depth/image_raw",  # Make sure this topic exists
            Image,
            self.depth_image_callback
        )

    def detection_callback(self, msg):
        for obj in msg.yolo_object:
            if obj.name == "person" or obj.name == "chair" or obj.name == "couch":
                self.min_x = obj.xmin
                self.max_x = obj.xmax
                self.min_y = obj.ymin
                self.max_y = obj.ymax

                self.centroid = int((self.min_x + self.max_x) / 2), int((self.min_y + self.max_y) / 2)
                self.human_u, self.human_v, self.human_depth = self.pixel_2_depth(self.centroid)

                print(self.pixel_2_depth(self.centroid))

                # Create a PointStamped message
                centroid_msg = PointStamped()
                centroid_msg.header.stamp = rospy.Time.now()
                centroid_msg.header.frame_id = "camera_depth_frame"
                centroid_msg.point.x = self.human_depth
                centroid_msg.point.y = -self.human_u
                centroid_msg.point.z = -self.human_v

                self.centroid_pub.publish(centroid_msg)

                rospy.loginfo(f"Published centroid for {obj.name} with ID {obj.id}: (depth,x = {centroid_msg.point.x}, y = {centroid_msg.point.y}, z = {centroid_msg.point.z})")

                # Draw centroid on depth image (for visualization)
                if self.latest_depth_img is not None:
                    display_img = self.latest_depth_img.copy()
                    cv2.circle(display_img, self.centroid, 5, (0, 0, 255), -1)  # Red dot
                    cv2.imshow("Centroid Tracking", display_img)
                    cv2.waitKey(1)

    def pixel_2_depth(self, coordinates):
        if hasattr(self, 'point_list') and len(self.point_list) > 0:
            index = coordinates[1] * 640 + coordinates[0]
            if index < len(self.point_list):
                u = self.point_list[index][0]
                v = self.point_list[index][1]
                depth = self.point_list[index][2]
                return (u, v, depth)
        return (0, 0, 0)

    def depth_callback(self, data):
        self.point_list = list(pc2.read_points(data, skip_nans=False))

    def depth_image_callback(self, img_msg):
        # Convert depth image to CV2 format
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
        # Normalize for visualization
        cv_image_norm = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
        cv_image_norm = np.uint8(cv_image_norm)
        self.latest_depth_img = cv2.cvtColor(cv_image_norm, cv2.COLOR_GRAY2BGR)

    def __del__(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node('centroid_tracker')

    CentroidTrackerNode()
    rospy.spin()
