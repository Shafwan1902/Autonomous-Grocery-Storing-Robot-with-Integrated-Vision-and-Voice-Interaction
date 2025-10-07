#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray
from tf.transformations import euler_from_quaternion
import tf
import actionlib
import math
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class StoreGroceriesManager:
    def __init__(self):
        rospy.init_node('store_groceries_manager', anonymous=True)

        self.text = ""
        self.home_pose = None
        self.current_pose = None
        self.tolerance = 0.4

        # Placeholder coordinates
        self.table_pos = (-1.6 , 1.11 )
        self.table_angle = 0.0
        self.cabinet_pos = (-2.53, -0.90 )
        self.cabinet_angle = 90.0

        self.detected_objects = {}

        self.yolo_class_names = {
            0: 'person', 39: 'bottle', 41: 'cup', 45: 'bowl',
            80: 'milk', 46: 'banana', 47: 'apple', 48: 'sandwich',
            49: 'orange', 50: 'broccoli', 51: 'carrot',
            # Add custom grocery items if needed
        }

        # ROS I/O
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.text2speach_pub = rospy.Publisher("text2speach", String, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseStamped, self.pose_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.speach2text_sub = rospy.Subscriber("speach2text", String, self.speach2text_callback)
        self.yolo_sub = rospy.Subscriber("/yolov8/detections", Detection2DArray, self.yolo_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def odom_callback(self, msg):
        if self.home_pose is None:
            self.home_pose = msg.pose.pose

    def speach2text_callback(self, msg):
        self.text = msg.data.lower()

    def yolo_callback(self, msg):
        for detection in msg.detections:
            class_id = detection.results[0].id
            confidence = detection.results[0].score
            if class_id in self.yolo_class_names:
                class_name = self.yolo_class_names[class_id]
                if confidence > 0.6:
                    self.detected_objects[class_name] = [confidence, detection.bbox]

    def voice_logger(self, text_to_speak):
        rospy.loginfo(f"SPEAKING: {text_to_speak}")
        msg = String()
        msg.data = text_to_speak
        self.text2speach_pub.publish(msg)

    def calculate_distance_to_goal(self, goal_x, goal_y):
        if self.current_pose:
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            return math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return float('inf')

    def move2goal(self, location, angle, wait=True):
        x, y = location
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(angle))
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(goal)

        if wait:
            while not rospy.is_shutdown():
                distance = self.calculate_distance_to_goal(x, y)
                if distance <= self.tolerance:
                    self.client.cancel_goal()
                    rospy.loginfo("Reached goal.")
                    break
                rospy.sleep(0.5)

    def run(self):
        while self.home_pose is None and not rospy.is_shutdown():
            rospy.loginfo("Waiting for home pose...")
            rospy.sleep(1)

        self.voice_logger("Store groceries task is ready. Say 'start' to begin.")
        while not rospy.is_shutdown():
            if self.text == "start":
                self.text = ""
                break
            rospy.sleep(0.5)

        for i in range(5):
            self.voice_logger(f"Now processing item {i+1} of 5")

            self.voice_logger("Heading to the table.")
            self.move2goal(self.table_pos, self.table_angle)
            rospy.sleep(2)

            self.voice_logger("Scanning for object...")
            self.detected_objects.clear()
            rospy.sleep(3)  # Simulate detection time

            if not self.detected_objects:
                self.voice_logger("No object found. Skipping.")
                continue

            item_list_str = ", ".join(self.detected_objects.keys())
            self.voice_logger(f"Detected: {item_list_str}")
            rospy.sleep(2)

            self.voice_logger("Heading to the cabinet.")
            self.move2goal(self.cabinet_pos, self.cabinet_angle)
            rospy.sleep(2)

        self.voice_logger("All groceries processed. Task complete.")
        rospy.sleep(1)
        self.voice_logger("Returning to start.")
        self.move2goal((self.home_pose.position.x, self.home_pose.position.y), 0.0)
        self.voice_logger("I have returned. Goodbye.")

if __name__ == '__main__':
    try:
        manager = StoreGroceriesManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass