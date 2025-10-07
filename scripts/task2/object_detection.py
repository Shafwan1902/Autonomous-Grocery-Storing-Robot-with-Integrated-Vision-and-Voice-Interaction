#! /usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2023 Jupiter Robot Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from jupiterobot2_msgs.msg import YoloMsg, ObjectMsg
import sys

sys.path.insert(0, '/home/mustar/catkin_ws/src/jupiterobot2/jupiterobot2_vision/jupiterobot2_vision_yolov5/scripts')
from detector import Detector


class ObjectDetection():

    def __init__(self):
        rospy.init_node('object_detection_human')

        self.detector = Detector()

        self.bridge = CvBridge()

        rospy.Subscriber('camera/color/image_raw', Image, self.image_callback, queue_size=1, buff_size=10000000)

        # 定义数据的发布
        self.info_pub = rospy.Publisher('yolo/object_info', YoloMsg, queue_size=1)
        self.yolo_msgs_pub = YoloMsg()

        self.pTime = 0


    def image_callback(self, msg):
        try:
            img0 = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        except CvBridgeError as e:
            rospy.logwarn(str(e))
            return

        img1, result = self.detector.detect(img0.copy()) # img0 to img1 (remove show object)
        cTime = time.time()
        fps = 1 / (cTime - self.pTime)
        self.pTime = cTime

        cv2.putText(img0, f"FPS:{fps:.1f}", (20, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 3)

        

        # 临时的数据类型存放在列表
        object_list = []
        for i, r in enumerate(result):
            if r.name == "person" or r.name == "couch" or r.name == "chair" and r.conf > 0.7:
                # rospy.loginfo(
                #     f'{i}: ({r.u1}, {r.v1}) ({r.u2}, {r.v2})' +
                #     f' {r.name}, {r.conf:.3f}')
                object_msgs_pub = ObjectMsg()
                object_msgs_pub.id = i
                object_msgs_pub.name = r.name
                object_msgs_pub.conf = r.conf
                object_msgs_pub.xmin = r.u1
                object_msgs_pub.ymin = r.v1
                object_msgs_pub.xmax = r.u2
                object_msgs_pub.ymax = r.v2
                object_list.append(object_msgs_pub)

                cv2.rectangle(img0, (int(r.u1),int(r.v1)), (int(r.u2),int(r.v2)), (255,0,0), 3) # show persom

                
# --------------------show result-----------------
        # 将列表的数据类型赋值给自定义的数据类型
        cv2.imshow('result', img0) 
        cv2.waitKey(1)
        self.yolo_msgs_pub = object_list
        self.info_pub.publish(self.yolo_msgs_pub)
# --------------------show result-----------------

if __name__ == "__main__":
    ObjectDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

