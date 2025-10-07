#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from visual_kinematics.RobotSerial import *
import numpy as np
from math import pi

class ArmController:
    def __init__(self):
        rospy.init_node('arm_ik_controller', anonymous=True)

        # Publishers for each joint
        self.joint1_pub = rospy.Publisher('/arm1_joint/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/arm2_joint/command', Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher('/arm3_joint/command', Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher('/arm4_joint/command', Float64, queue_size=10)

        # Define your DH parameters
        self.dh_params = np.array([
            [0.043, 0.0, 0.5 * pi, 0.0],
            [0.0, 0.105, 0.0, 0.0],
            [0.0, 0.105, 0.0, 0.0],
            [0.0, 0.063, 0.0, 0.0]
        ])

        # Initialize the robot model
        self.robot_arm = RobotSerial(self.dh_params)

        # Subscribe to 3D point from the depth extractor
        rospy.Subscriber('/object_position', Point, self.point_callback)

        rospy.loginfo("✅ Arm IK Controller initialized.")

    def point_callback(self, point_msg):
        rospy.loginfo(f"Received 3D point: x={point_msg.x:.3f}, y={point_msg.y:.3f}, z={point_msg.z:.3f}")

        # Prepare 3D point
        xyz = np.array([[point_msg.x], [point_msg.y], [point_msg.z]])

        # Assume zero orientation (can be improved later)
        abc = np.array([0, 0, 0])  # Euler angles
        target_frame = Frame.from_euler_3(abc, xyz)

        try:
            self.robot_arm.inverse(target_frame)
            thetas = self.robot_arm.axis_values

            # Publish to each joint
            self.joint1_pub.publish(Float64(thetas[0]))
            self.joint2_pub.publish(Float64(-(thetas[1] - 0.5 * pi)))
            self.joint3_pub.publish(Float64(-thetas[2]))
            self.joint4_pub.publish(Float64(-thetas[3]))

            rospy.loginfo(f"Sent joint angles: {np.round(thetas, 3)}")

        except Exception as e:
            rospy.logerr(f"Inverse kinematics failed: {str(e)}")

if __name__ == '__main__':
    try:
        ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
