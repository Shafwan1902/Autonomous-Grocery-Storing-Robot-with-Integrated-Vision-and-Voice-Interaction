#!/usr/bin/env python3
import rospy
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
from my_robot_msgs.msg import ObjectMsg, Yolov8Msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ObjectDetector:
    def __init__(self, model_path="/home/mustar/catkin_ws/src/teamA_robocup2025/yolov8m.pt"):
        rospy.init_node('object_detection', anonymous=True)

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.target_class = "bottle"  # ✅ Change this if you want a different object filter

        # ROS Subscribers
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1, buff_size=10000000)

        # ROS Publishers
        self.info_pub = rospy.Publisher('/yolov8/object_info', Yolov8Msg, queue_size=1)
        self.bbox_pub = rospy.Publisher('/detected_object_bbox', Int32MultiArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

        rospy.loginfo("✅ Hybrid YOLOv8 Object Detector initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(str(e))
            return

        object_list = []
        results = self.model(frame)

        for i, result in enumerate(results):
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if conf < 0.5 or label != self.target_class:
                    continue  # Skip non-target or low-confidence objects

                # Draw on image
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                text = f"{label}: {conf:.2f}"
                cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Populate custom message
                object_msg = ObjectMsg()
                object_msg.id = i
                object_msg.name = label
                object_msg.conf = conf
                object_msg.xmin = x1
                object_msg.ymin = y1
                object_msg.xmax = x2
                object_msg.ymax = y2
                object_list.append(object_msg)

                # Publish bounding box center
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)
                bbox_msg = Int32MultiArray(data=[x_center, y_center])
                self.bbox_pub.publish(bbox_msg)

                # Publish RViz Marker
                marker = Marker()
                marker.header.frame_id = "camera_depth_frame"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = marker.scale.y = marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.position = Point(x_center, y_center, 0)
                marker.id = i
                self.marker_pub.publish(marker)

        # Publish all objects detected
        yolo_msg = Yolov8Msg()
        yolo_msg.objects = object_list
        self.info_pub.publish(yolo_msg)

        # Show frame
        cv2.imshow('YOLOv8 Detection', frame)
        cv2.waitKey(1)

if __name__ == "__main__":
    detector = ObjectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
