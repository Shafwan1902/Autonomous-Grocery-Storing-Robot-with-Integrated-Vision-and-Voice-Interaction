#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Float64
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker
import time
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Manager:
    def __init__(self):
        rospy.init_node('carry_my_luggage_manager', anonymous=True)

        # Subscribers
        self.human_state_sub = rospy.Subscriber(
            "manager/human_state",
            Bool,
            self.human_state_callback
        )

        self.speach2text_sub = rospy.Subscriber(
            "speach2text",
            String,
            self.speach2text_callback
        )


        self.goal_status_sub = rospy.Subscriber(
            '/move_base/status',
            GoalStatusArray,
            self.goal_status_callback
        )
        
        self.marker_sub = rospy.Subscriber(
            "/human_visualization",
            Marker,
            self.marker_callback
        )

        self.odom_sub = rospy.Subscriber(
            '/odom',
            Odometry,
            self.odom_callback
        )

        # publishers
        self.text2speach_pub = rospy.Publisher(
            "text2speach",
            String,
            queue_size=10
        )

        self.head_joint_pub = rospy.Publisher(
            "/head_joint/command",
            Float64,
            queue_size=10
        )

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=10
        )

        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=10
        )

        # State variables
        self.there_is_human = False
        self.rate = rospy.Rate(1)
        self.voice_log = True
        self.start = False
        self.goal_reached = None
        self.x_poss = None
        self.y_poss = None
        self.yes_cary = False
        
        self.host = [ None, None] 
        self.guest1 = [ None, None] 
        self.guest2 = [ None, None] 
        self.guest3 = [ None, None]

        self.current_yaw = 0.0
        self.target_yaw = None
        self.rotating = False
       

    def goal_status_callback(self, msg):
        for status in msg.status_list:
            if status.status == 3:
                self.goal_reached = True
            elif status.status == 4:
                self.goal_reached = False

    def marker_callback(self, data):
        self.x_poss = data.pose.position.x
        self.y_poss = data.pose.position.y

    def odom_callback(self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)

    def head_shake(self):
        self.head_controller(0.0)
        time.sleep(0.5)
        self.head_controller(0.1)
        time.sleep(2)

    def head_controller(self, angle):
        msg = Float64()
        msg.data = angle
        self.head_joint_pub.publish(msg)

    def voice_logger(self, text):
        if self.voice_log:
            msg = String()
            msg.data = text
            self.text2speach_pub.publish(msg)

    def speach2text_callback(self, msg):
        text = msg.data.lower()
        if text == 'active':
            self.start = True
        elif text == 'yes':
            self.yes_cary = True

    def human_state_callback(self, msg):
        self.there_is_human = msg.data

    def move2guest(self, person):
        msg = PoseStamped()
        msg.header.frame_id = "camera_depth_frame"
        msg.pose.position.x = person[0] - 0.7
        msg.pose.position.y = person[1]
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def save_position(self, person):
        person[0] = self.x_poss
        person[1] = self.y_poss

    def rotate(self, degrees, angular_velocity):
        twist = Twist()
        twist.angular.z = angular_velocity
        rotation_time = (degrees * math.pi / 180.0) / angular_velocity
        start_time = rospy.Time.now().to_sec()
        self.target_yaw = self.current_yaw + math.radians(degrees)
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))
        self.rotating = True
        
        rospy.loginfo("Starting rotation...")
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown() and self.rotating:
            current_yaw_normalized = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
            
            if abs(current_yaw_normalized - self.target_yaw) < 0.05:  # Threshold of 0.05 radians (~2.86 degrees)
                self.rotating = False
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Rotation complete")
                break
            else:
                self.cmd_vel_pub.publish(twist)
            
            rate.sleep()

    # def rotate_180_degrees(self):
    #     self.rotate(360, 1.0)  # Rotate 360 degrees at 1.0 rad/s to reset orientation
    #     self.rotate(180, 1.0)  # Rotate 180 degrees at 1.0 rad/s

    def run(self):
        self.head_controller(0.0)
        while not rospy.is_shutdown():
            # start
            while not rospy.is_shutdown():
                if self.start:
                    self.head_controller(angle=0.1)
                    self.voice_logger("activated")
                    break

            # take the host position
            while not rospy.is_shutdown():
                if self.there_is_human:
                    self.save_position(self.host)
                    self.voice_logger("Saving position for host")
                    break
                else:
                    self.voice_logger("looking for host")
                    self.head_shake()

            rospy.sleep(5)

            # rotate
            self.rotate(180, 1.0)

            # looking for guest1
            while not rospy.is_shutdown():
                if self.there_is_human:
                    self.save_position(self.guest1)
                    self.voice_logger("i can see a guest")
                    break
                else:
                    self.voice_logger("looking for guest")
                    self.head_shake()

            # lock selected guest1 location
            while not rospy.is_shutdown():
                try:
                    # move to guest
                    self.move2guest(self.guest1)
                    self.voice_logger("moving to the guest")
                    break
                except:
                    self.head_shake()

            # waiting for goal reaching...
            while not rospy.is_shutdown():
                if self.goal_reached is False:
                    self.voice_logger("trying to move to the guest")
                    self.move2guest(self.guest1)
                elif self.goal_reached is True:
                    self.voice_logger("done approaching the guest")
                    break
            
            rospy.sleep(10)
            # asking guest1 name
            self.voice_logger("Hello, how are you? What is your name?")

            # scan the items near the person1
            rospy.sleep(5)
            # go to host
            while not rospy.is_shutdown():
                try:
                    # move to guest
                    self.voice_logger("Moving to host")
                    self.move2guest(self.host)
                    break
                except:
                    self.head_shake()

            # waiting for goal reaching...
            while not rospy.is_shutdown():
                if self.goal_reached is False:
                    self.voice_logger("trying to move to the guest")
                    self.move2guest(self.host)
                elif self.goal_reached is True:
                    self.voice_logger("done approaching the guest")
                    break
            

            # tell the host

            # got to next location

            # looking for guest2

            # lock selected guest2 location

            # waiting for goal reaching...

            # asking guest2 name
            # self.voice_logger("Hello, how are you? What is your name?")

            # scan the items near the guest2

            # go to host
            # self.voice_logger("Moving to host")
            # self.request_saved_position("host")

            # tell the host

            # looking for guest3

            # lock selected guest3 location

            # waiting for goal reaching...

            # asking guest3 name
            # self.voice_logger("Hello, how are you? What is your name?")

            # scan the items near the guest3

            # go to host

            # tell the host

            # hopfully that's all

            self.start = False
                
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Manager()
        node.run()
    except rospy.ROSInterruptException:
        pass
