#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from jupiterobot2_msgs.msg import YoloMsg     # ObjectMsg
from geometry_msgs.msg import PointStamped
# from dynamixel_msgs.msg import JointState

class CentroidTrackerNode:

    def __init__(self):

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

        # self.head_state_sub = rospy.Subscriber(
        #     "/head_joint/state",
        #     JointState,
        #     self.head_state_callback
        # )

        # Publisher for centroids
        self.centroid_pub = rospy.Publisher(
            'human_centroids', 
            PointStamped, 
            queue_size=10
        )

        # self.head_joint_state = 0.0

    def detection_callback(self, msg):
        for obj in msg.yolo_object:
            # print(obj.name)
            if obj.name == "person" or obj.name == "chair" or obj.name == "couch":
                # Calculate the centroid of the bounding box
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
                centroid_msg.header.frame_id = "camera_depth_frame"  # Change "camera_link" to your appropriate frame
                centroid_msg.point.x = self.human_depth
                centroid_msg.point.y = -self.human_u
                centroid_msg.point.z = -self.human_v

                # Publish the centroid
                self.centroid_pub.publish(centroid_msg)


                rospy.loginfo(f"Published centroid for {obj.name} with ID {obj.id}: (depth,x = {centroid_msg.point.x}, y = {centroid_msg.point.y}, z = {centroid_msg.point.z})")


    def pixel_2_depth(self, coordinates):
        index = coordinates[1] * 640 + coordinates[0]
        u = self.point_list[index][0]
        v = self.point_list[index][1]
        depth = self.point_list[index][2]
        return (u, v, depth)
    
    def depth_callback(self, data):
        self.point_list = list(pc2.read_points(data, skip_nans=False))

    # def head_state_callback(self, msg):
    #     self.head_joint_state = msg.current_pos
    

if __name__ == "__main__":
    rospy.init_node('centroid_tracker')

    CentroidTrackerNode()
    rospy.spin()
