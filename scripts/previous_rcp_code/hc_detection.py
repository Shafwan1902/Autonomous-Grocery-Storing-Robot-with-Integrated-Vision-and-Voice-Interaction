#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#maintanined by DSYTE

import rospy
import cv2
import numpy as np
from std_msgs.msg import UInt16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

# Adjust the path as necessary to locate the Detector class
sys.path.insert(0, '/home/mustar/catkin_ws/src/jupiterobot2/jupiterobot2_vision/jupiterobot2_vision_yolov5/scripts')
from detector import Detector

class ObjectDetection:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('hc_detection')

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

        # Define the object names to detect
        self.object_names = ['person', 'chair','couch']  # Updated to detect humans and chairs

        # Multiplier for depth values (adjust as necessary)
        self.depth_image_multiplier = 1

        # Initialize storage for detected chairs and humans
        self.chairs = []  # List to store detected chairs
        self.humans = []  # List to store detected humans

        # Define the loop rate (10 Hz)
        self.rate = rospy.Rate(10)

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV image (BGR8 format)
            self.img_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"CvBridge Error: {e}")
            return

        # Perform object detection using the Detector
        _, detections = self.detector.detect(self.img_color.copy())

        # Reset detected chairs and humans lists
        self.chairs = []
        self.humans = []

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
        Determine if chairs are occupied by checking for overlapping with humans.
        """
        for chair in self.chairs:
            chair_bbox = chair['bbox']
            chair_center = (
                (chair_bbox[0] + chair_bbox[2]) // 2,
                (chair_bbox[1] + chair_bbox[3]) // 2
            )

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
                    chair['occupied'] = True
                    break  # No need to check other humans for this chair

    def visualize_detections(self):
        """
        Visualize detected chairs and humans with occupancy status.
        """
        img = self.img_color.copy()

        # Draw chairs with occupancy status
        for chair in self.chairs:
            x1, y1, x2, y2 = chair['bbox']
            status = "Occupied" if chair['occupied'] else "Unoccupied"
            color = (0, 0, 255) if chair['occupied'] else (0, 255, 0)  # Red for occupied, Green for unoccupied

            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

            # Put label
            cv2.putText(img, status, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Optionally, draw humans
        for human in self.humans:
            x1, y1, x2, y2 = human['bbox']
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue boxes for humans

        # Display the image
        cv2.imshow("Detection Results", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        od = ObjectDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
