#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from jupiterobot2_msgs.msg import Mediapipe_Pose
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Bool
import numpy as np
from dynamixel_msgs.msg import JointState
import tf

class HumanRangeFinder:

    def __init__(self): 
        

        self.pose_sub = rospy.Subscriber(
            "/mediapipe/pose",
            Mediapipe_Pose,
            self.pose_callback   
        )

        self.head_state_sub = rospy.Subscriber(
            "/head_joint/state",
            JointState,
            self.head_state_callback
        )

        self.depth_sub = rospy.Subscriber(
            "/camera/depth/points",
            PointCloud2,
            self.depth_callback   
        )

        self.marker_pub = rospy.Publisher(
            "/human_visualization",
            Marker,
            queue_size=10
        )


        self.selected_luggage_pub = rospy.Publisher(
            "/selected_luggage",
            String,
            queue_size=10
        )

        self.state_pub = rospy.Publisher(
            "manager/human_state",
            Bool,
            queue_size=10
        )

        self.selected_luggage = String()
        self.human_location = None
        self.human_range = 4.0

        self.head_joint_state = 0.0


    def head_state_callback(self, msg):
        self.head_joint_state = msg.current_pos
        
     
    def pose_callback(self, data):
        
        try:
            self.left_shoulder = (data.left_shoulder.x,
                                    data.left_shoulder.y)
            
            self.right_shoulder = (data.right_shoulder.x,
                                    data.right_shoulder.y)
            
            self.left_hip = (data.left_hip.x,
                                data.left_hip.y)
            
            self.right_hip=(data.right_hip.x,
                            data.right_hip.y)
            
            self.left_elbow = (data.left_elbow.x,
                                data.left_elbow.y)
            
            self.right_elbow = (data.right_elbow.x,
                                data.right_elbow.y)
            
            self.left_wrist = (data.left_wrist.x,
                                data.left_wrist.y)
            
            self.right_wrist = (data.right_wrist.x,
                                data.right_wrist.y)
            


            # find center of human chest
            self.box = [self.left_shoulder , self.right_shoulder, 
                        self.left_hip      , self.right_hip      ]
            
            xs, ys = list(zip(*self.box))

            self.min_x = min(xs)
            self.max_x = max(xs)
            self.min_y = min(ys)
            self.max_y = max(ys)

            self.centroid = int((self.min_x + self.max_x) /2), int((self.min_y + self.max_y) /2)
            
            self.centroid_compare = self.centroid[0]
            self.leftw_x = self.left_wrist[0]
            self.rightw_x = self.right_wrist[0]

            if self.leftw_x > self.centroid_compare + 80:
                self.direction = "left"
                self.selected_luggage.data = "left"
            elif self.rightw_x < self.centroid_compare - 80:
                self.direction = "right"
                self.selected_luggage.data = "right"
                
            else:
                self.direction = None
                self.selected_luggage.data = "none"

            
            self.human_u, self.human_v, self.human_depth = self.pixel_2_depth(self.centroid)
            result = np.array([self.human_u, self.human_v, self.human_depth])
            state = Bool()
            if not np.isnan(result[0]):
                if self.human_depth <= self.human_range:
                    state.data = True
                    self.selected_luggage_pub.publish(self.selected_luggage)
                    self.visualization()
            else:
                state.data = False
            self.state_pub.publish(state)
          
            
        except:
            pass
    
    
    def pixel_2_depth(self, coordinates):
        index = coordinates[1] * 640 + coordinates[0]
        u = self.point_list[index][0]
        v = self.point_list[index][1]
        depth = self.point_list[index][2]
        return (u, v, depth)

    
    def depth_callback(self, data):
        self.point_list = list(pc2.read_points(data, skip_nans=False))




    def visualization(self):
        body_marker = Marker()
        

        body_marker.header.frame_id = 'camera_depth_frame'
        body_marker.header.stamp = rospy.Time.now()
        body_marker.ns = 'body'
        body_marker.type = 1 # Cube
        body_marker.scale.x = 0.5
        body_marker.scale.y = 0.5
        body_marker.scale.z = 2.0
        body_marker.pose.position.x = self.human_depth
        body_marker.pose.position.y = - self.human_u
        body_marker.pose.position.z = - self.human_v

        quaternion = tf.transformations.quaternion_from_euler(0.0, - self.head_joint_state, 0.0)

        body_marker.pose.orientation.x = 0.0
        body_marker.pose.orientation.y = quaternion[1]
        body_marker.pose.orientation.z = 0.0
        body_marker.pose.orientation.w = 1.0
        body_marker.color.r = 0.0
        body_marker.color.g = 1.0
        body_marker.color.b = 1.0
        body_marker.color.a = 0.5

        self.marker_pub.publish(body_marker)



if __name__ == '__main__':
    rospy.init_node('human_range_finder')
    
    HumanRangeFinder()
    rospy.spin()


