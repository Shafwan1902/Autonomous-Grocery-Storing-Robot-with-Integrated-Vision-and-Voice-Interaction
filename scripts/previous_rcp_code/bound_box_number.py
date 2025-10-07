#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Maintained by DSYTE

import rospy
import cv2
import numpy as np
import math  # Import to calculate theta
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, String, Float64, UInt16MultiArray

# Adjust the path as necessary to locate the Detector class
sys.path.insert(0, '/home/mustar/catkin_ws/src/jupiterobot2/jupiterobot2_vision/jupiterobot2_vision_yolov5/scripts')
from detector import Detector

class ObjectDetection:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_detection')

        # Initialize the Detector
        self.detector = Detector()

        # Initialize CvBridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to color and depth image topics
        rospy.Subscriber('camera/color/image_raw', Image, self.color_image_callback)
        rospy.Subscriber('camera/depth/image_raw', Image, self.depth_image_callback)

        # Publisher for detection results
        self.detection_pub = rospy.Publisher(
            '/detection_results',
            UInt16MultiArray,
            queue_size=10
        )

        self.theta_pub = rospy.Publisher(
            '/theta_result',
            Float64,
            queue_size=10
        )

        self.odom_sub = rospy.Subscriber(
            "/odom", 
            Odometry,
            self.odom_callback
        )

        # Define the object names to detect
        self.object_names = ['person', 'chair', 'couch']  # Updated to include couch

        # Multiplier for depth values (adjust as necessary)
        self.depth_image_multiplier = 1

        # Initialize storage for detected chairs, humans, and couches
        self.chairs = []  # List to store detected chairs
        self.humans = []  # List to store detected humans
        self.couches = []  # List to store detected couches

        # Define the loop rate (10 Hz)
        self.rate = rospy.Rate(10)
    
    def odom_callback(self, msg):
        self.robot_odom = msg.pose
        self.x_robot = self.robot_odom.position.x
        self.y_robot = self.robot_odom.position.y
        self.rot = self.robot_odom.orientation
        (self.roll, self.pitch, self.theta) = euler_from_quaternion((self.rot.x, self.rot.y, self.rot.z, self.rot.w))
    
    def cal_angle(self, x1, y1):
        self.x_cek = x1 - self.x_robot
        self.y_cek = y1 - self.y_robot
        self.thetas = math.atan2(self.y_cek, self.x_cek)
        return self.thetas
    
    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV image (BGR8 format)
            self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridge Error: {e}")
            return

        # Perform object detection using the Detector
        _, detections = self.detector.detect(self.img_color.copy())

        # Reset detected chairs, humans, and couches lists
        self.chairs = []
        self.humans = []
        self.couches = []

        # Initialize the message to publish detection results
        detection_msg = UInt16MultiArray()
        detection_msg.data = []

        # Process each detected object
        for obj in detections:
            if obj.name in self.object_names:
                # Extract bounding box coordinates
                x1, y1, x2, y2 = int(obj.u1), int(obj.v1), int(obj.u2), int(obj.v2)

                # Create a dictionary for the detected object
                detected_obj = {
                    'name': obj.name,
                    'bbox': (x1, y1, x2, y2)
                }

                if obj.name == 'chair':
                    # Store chair information
                    detected_obj['occupied'] = False  # Initialize as unoccupied
                    self.chairs.append(detected_obj)
                elif obj.name == 'person':
                    # Store human information
                    self.humans.append(detected_obj)
                elif obj.name == 'couch':
                    # Store couch information
                    detected_obj['centroid'] = self.calculate_centroid((x1, y1, x2, y2))
                    self.couches.append(detected_obj)

        # Determine occupancy status
        self.determine_occupancy()

        # Prepare detection message data
        for chair in self.chairs:
            x1, y1, x2, y2 = chair['bbox']
            occupied = 1 if chair['occupied'] else 0  # 1 for occupied, 0 for unoccupied
            msg = [x1, y1, x2, y2, occupied]
            detection_msg.data.extend(msg)

        # Publish the detection results
        self.detection_pub.publish(detection_msg)

        # Visualize detections
        self.visualize_detections()

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV image (depth format) and apply multiplier
            self.img_depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough') * self.depth_image_multiplier
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridge Error: {e}")
            return

    def determine_occupancy(self):
        """
        Determine if chairs are occupied by checking for overlapping with humans,
        and check if couch sections (left, middle, right) are occupied.
        Print the coordinates (x, y, theta) of unoccupied sections.
        """
        # Counter for unoccupied objects
        unoccupied_counter = 1
        
        # First, determine occupancy for chairs
        for chair in self.chairs:
            chair_bbox = chair['bbox']
            chair_center = (
                (chair_bbox[0] + chair_bbox[2]) // 2,
                (chair_bbox[1] + chair_bbox[3]) // 2
            )

            occupied = False

            for human in self.humans:
                human_bbox = human['bbox']
                human_center = (
                    (human_bbox[0] + human_bbox[2]) // 2,
                    (human_bbox[1] + human_bbox[3]) // 2
                )

                # Simple overlap check: if centers are close enough
                distance = np.linalg.norm(np.array(chair_center) - np.array(human_center))
                threshold = max(chair_bbox[2] - chair_bbox[0], chair_bbox[3] - chair_bbox[1]) // 2

                if distance < threshold:
                    # Assuming the human is sitting on the chair
                    occupied = True
                    break  # No need to check other humans for this chair

            chair['occupied'] = occupied

            if not occupied:
                chair_centroid = self.calculate_centroid(chair_bbox)
                self.print_coordinates(chair_centroid, 'chair', unoccupied_counter)
                unoccupied_counter += 1

        # Now, check occupancy for couches (left, middle, right)
        for couch in self.couches:
            x1, y1, x2, y2 = couch['bbox']
            width = x2 - x1

            # Define the couch sections
            left_section = (x1, y1, x1 + width // 3, y2)
            middle_section = (x1 + width // 3, y1, x1 + 2 * width // 3, y2)
            right_section = (x1 + 2 * width // 3, y1, x2, y2)

            # Initialize occupancy status for the three sections
            couch['left_occupied'] = False
            couch['middle_occupied'] = False
            couch['right_occupied'] = False

            for human in self.humans:
                human_bbox = human['bbox']
                human_center = (
                    (human_bbox[0] + human_bbox[2]) // 2,
                    (human_bbox[1] + human_bbox[3]) // 2
                )

                # Check which section the human is overlapping with
                if self.is_point_in_bbox(human_center, left_section):
                    couch['left_occupied'] = True
                    left_centroid = self.calculate_centroid(left_section)
                    self.print_coordinates(left_centroid, 'left', unoccupied_counter)
                    unoccupied_counter += 1
                elif self.is_point_in_bbox(human_center, middle_section):
                    couch['middle_occupied'] = True
                    middle_centroid = self.calculate_centroid(middle_section)
                    self.print_coordinates(middle_centroid, 'middle', unoccupied_counter)
                    unoccupied_counter += 1
                elif self.is_point_in_bbox(human_center, right_section):
                    couch['right_occupied'] = True
                    right_centroid = self.calculate_centroid(right_section)
                    self.print_coordinates(right_centroid, 'right', unoccupied_counter)
                    unoccupied_counter += 1

            # Print coordinates for sections that are not occupied
            if not couch['left_occupied']:
                left_centroid = self.calculate_centroid(left_section)
                self.print_coordinates(left_centroid, 'left', unoccupied_counter)
                unoccupied_counter += 1
            if not couch['middle_occupied']:
                middle_centroid = self.calculate_centroid(middle_section)
                self.print_coordinates(middle_centroid, 'middle', unoccupied_counter)
                unoccupied_counter += 1
            if not couch['right_occupied']:
                right_centroid = self.calculate_centroid(right_section)
                self.print_coordinates(right_centroid, 'right', unoccupied_counter)
                unoccupied_counter += 1

    def is_point_in_bbox(self, point, bbox):
        """
        Check if a point (x, y) is inside a bounding box.
        """
        x, y = point
        x1, y1, x2, y2 = bbox
        return x1 <= x <= x2 and y1 <= y <= y2

    def calculate_centroid(self, bbox):
        """
        Calculate the centroid of a bounding box.
        """
        x1, y1, x2, y2 = bbox
        centroid = ((x1 + x2) // 2, (y1 + y2) // 2)
        return centroid

    def print_coordinates(self, centroid, object_type, unoccupied_counter):
        """
        Print the coordinates (x, y, theta) for the occupied or unoccupied object (chair or couch section).
        """
        x, y = centroid
        angle = self.cal_angle(x, y)
        angle_pubs = Float64()
        angle_pubs.data = angle
        self.theta_pub.publish(angle_pubs)

        if object_type in ['chair', 'left', 'middle', 'right']:
            label = f"{object_type.capitalize()} {unoccupied_counter}"
        else:
            label = object_type.capitalize()

        print(f"{label} is unoccupied. Coordinates: x = {x}, y = {y}, theta = {angle:.2f}")

    def visualize_detections(self):
        """
        Visualize detected chairs, humans, and couches with occupancy status.
        """
        img = self.img_color.copy()

        # First, draw chairs with occupancy status
        for chair in self.chairs:
            x1, y1, x2, y2 = chair['bbox']
            status = "Occupied" if chair['occupied'] else "Unoccupied"
            color = (0, 0, 255) if chair['occupied'] else (0, 255, 0)  # Red for occupied, green for unoccupied
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img, f"Chair ({status})", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Then, draw couches and their sections with occupancy status
        for couch in self.couches:
            x1, y1, x2, y2 = couch['bbox']
            width = x2 - x1

            # Draw left, middle, and right sections
            sections = [
                ('left', couch['left_occupied'], (x1, y1, x1 + width // 3, y2)),
                ('middle', couch['middle_occupied'], (x1 + width // 3, y1, x1 + 2 * width // 3, y2)),
                ('right', couch['right_occupied'], (x1 + 2 * width // 3, y1, x2, y2))
            ]

            for section_name, occupied, section_bbox in sections:
                x1_s, y1_s, x2_s, y2_s = section_bbox
                color = (0, 0, 255) if occupied else (0, 255, 0)  # Red for occupied, green for unoccupied
                status = "Occupied" if occupied else "Unoccupied"
                cv2.rectangle(img, (x1_s, y1_s), (x2_s, y2_s), color, 2)
                cv2.putText(img, f"Couch {section_name} ({status})", (x1_s, y1_s - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Display the image
        cv2.imshow('Detections', img)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # Create an instance of the ObjectDetection class
        object_detection = ObjectDetection()
        
        # Keep the node running until interrupted
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
