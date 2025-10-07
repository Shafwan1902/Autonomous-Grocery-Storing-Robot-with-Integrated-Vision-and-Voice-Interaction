#!/usr/bin/env python3

import rospy
import actionlib
import math
from std_msgs.msg import Bool, String, Float64, UInt16MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose


class Manager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)

        # Subscribers
        self.human_state_sub = rospy.Subscriber("manager/human_state", Bool, self.human_state_callback)
        self.speach2text_sub = rospy.Subscriber("speach2text", String, self.speach2text_callback)
        self.detection_sub = rospy.Subscriber('/detection_results', UInt16MultiArray, self.detection_callback)

        # Publishers
        self.text2speach_pub = rospy.Publisher("text2speach", String, queue_size=10)
        self.head_joint_pub = rospy.Publisher("/head_joint/command", Float64, queue_size=10)
        self.arm1_joint_pub = rospy.Publisher(
            "/arm1_joint/command",
            Float64,
            queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            "/odom", 
            Odometry,
            self.odom_callback
        )

        self.cmd_pub = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=10)

        self.angle_sub = rospy.Subscriber("/theta_result", Float64, self.angle_callback)
        self.human_angle_sub = rospy.Subscriber("/human_result", Float64, self.human_angle_callback)


        # State Variables
        self.there_is_human = False
        self.voice_log = True
        self.start = False
        self.current_guest = None
        self.current_favorite_drink = None
        self.current_host = "John"
        self.host_favorite_drink = "coffee"
        self.guest_count = 0  
        self.waiting_for_yes = False
        self.reached_goal = False
        self.offering_chair = False
        self.thank_you_received = False
        self.check_pos = False
        self.calling_help = False
        self.unoccupied_seat_count = 0 
        self.robot_odom = (0, 0)

        self.last_speech_input = ""

        self.guest_name = ["julia", "emma", "sara", "laura", "susan", "john", "lucas", "william", "kevin", "peter", "robin", "jeffery", "tan", "sarah", "adam"]

        self.guest_drink = ["hot chocolate", "coke", "soy", "water", "lime juice", "tea", "coffee", "soda", "milk", "latte"]

        self.unoccupied_seats = []
        self.guest_inside = [0,0,0,0,0]
        self.drink_inside = [0,0,0,0,0]
        self.rate = rospy.Rate(10)

        # Navigation client
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Waypoints
        self.waypoints = {
            # 'host_location': (1.3, 1.51, math.pi * 1),  # Host's location
            'host_location': (0.761,2.0, math.pi * 1),  # Host's location
            'origin': (1.09, 1.61, math.pi * 2)  # Origin location
        }

    def publish_coordinates(self, target_point):
        self.reached_goal = False
        rospy.loginfo(f"Moving to target coordinates: x={target_point.x}, y={target_point.y}, z={target_point.z}")
        self.move_to_goal(target_point.x, target_point.y, target_point.z)
        self.reached_goal = True

    def odom_callback(self, msg):
        self.robot_odom = msg.pose

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
        self.last_speech_input = msg.data.lower()
        rospy.loginfo(f"Received speech input: {self.last_speech_input}")

    def detection_callback(self, msg):
        self.unoccupied_seats.clear()

        num_chairs = 0
        num_humans = 0

        for i in range(0, len(msg.data), 5):
            x1, y1, x2, y2, occupied = msg.data[i:i+5]
            if occupied:
                num_humans += 1
            else:
                num_chairs += 1
                self.unoccupied_seats.append((x1, y1, x2, y2))

        self.unoccupied_seat_count = num_chairs
        self.there_is_human = num_humans > 0

    def human_state_callback(self, msg):
        self.there_is_human = msg.data

    def head_shake(self):
        self.head_controller(0.0)
        rospy.sleep(0.5)
        self.head_controller(0.1)
        rospy.sleep(2.0)

    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(*quaternion))

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()


    def turn(self, angle_degrees, direction):
        # Parameters
        angular_speed = 0.5  # rad/s
        angle_radians = math.radians(angle_degrees)
        duration = angle_radians / angular_speed
        

        # Create a Twist message
        turn_cmd = Twist()
        if direction == 'left':
            turn_cmd.angular.z = angular_speed
        elif direction == 'right':
            turn_cmd.angular.z = -angular_speed
        else:
            rospy.logerr("Invalid direction. Use 'left' or 'right'.")
            return

        # Publish the command for the required duration
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.cmd_pub.publish(turn_cmd)
            rospy.sleep(0.1)
        # Stop the robot
        turn_cmd.angular.z = 0
        self.check_pos = True
        self.cmd_pub.publish(turn_cmd)

    def move_forward(self, distance):
        # Parameters
        if distance < 0:
            speed = - 0.05
        else:
            speed = 0.05  # m/s

        duration = abs(distance) / abs(speed)
        # Create a Twist message
        move_cmd = Twist()
        move_cmd.linear.x = speed
        # Publish the command for the required duration
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < duration:
            self.cmd_pub.publish(move_cmd)
            rospy.sleep(0.1)
        # Stop the robot
        move_cmd.linear.x = 0
        self.cmd_pub.publish(move_cmd)




    def run(self):

        while not rospy.is_shutdown() :
            if self.last_speech_input == "help":
                self.calling_help = True
                break
            else:
                pass
            
        if self.calling_help:
            self.publish_coordinates((x,y,z))
            rospy.sleep(2)
            self.voice_logger("yes my host, do you need help?")
            
            
            rospy.sleep(10)

        if self.

        self.head_controller(0.0)
        self.pointing(0.0)

        rot_arm = -0.65
        num = 0

        while not rospy.is_shutdown():
            if self.there_is_human and self.guest_count < 3:
                if not self.start:
                    # self.head_shake()
                    rospy.loginfo("Human detected, waiting 2 seconds before greeting...")
                    rospy.sleep(2)
                    self.voice_logger("Hello, nice to meet you! May I know your name?")
                    while True:
                        rospy.sleep(1)
                        if self.last_speech_input in self.guest_name:
                            self.current_guest = self.last_speech_input
                            self.voice_logger(f"Nice to meet you, {self.current_guest}. What is your favorite drink?")
                            while True:
                                if self.last_speech_input in self.guest_drink:
                                    self.current_favorite_drink = self.last_speech_input
                                    self.guest_inside[num] = self.current_guest
                                    self.drink_inside[num] = self.current_favorite_drink
                                    self.voice_logger(f"Thank you for the information and follow on my left side to my host")
                                    print(self.guest_inside[0])
                                    rospy.sleep(5)
                                    self.move_forward(-0.1)
                                    rospy.sleep(2)
                                    self.turn(145, "left")
                                    rospy.sleep(3)
                                    # point = Point(*self.waypoints['host_location'])
                                    # self.publish_coordinates(point)
                                    break
                            self.start = True
                            num = num + 1
                            break

            elif self.check_pos and not self.offering_chair:
                # seat_message = f"There are {self.unoccupied_seat_count} seats available."
                self.voice_logger("Hi my host")
                rospy.sleep(2)
                self.pointing(1.45)
                # self.pointing_host()
                rospy.sleep(3)
                self.voice_logger(f"you have a guest named {self.current_guest}. {self.current_guest} favorite drink is {self.current_favorite_drink}.")

                if self.guest_inside[1] != 0 :
                    self.voice_logger(f"Hey {self.current_guest}, We have first guest named {self.guest_inside[0]}. {self.guest_inside[0]} favorite drink is {self.drink_inside[0]}.")

                # elif self.guest_inside[2] is not None and self.drink_inside[2] is not None:
                #     self.voice_logger(f"We have first visitor named {self.guest_inside[0]}. {self.guest_inside[0]} favorite drink is {self.drink_inside[0]}.")
                #     rospy.sleep(1)
                #     self.voice_logger(f"We have second visitor named {self.guest_inside[1]}. {self.guest_inside[1]} favorite drink is {self.drink_inside[1]}.")


                # self.voice_logger(f"{self.current_guest}, this is {self.current_host}, my host. {self.current_host}'s favorite drink is {self.host_favorite_drink}.")
                rospy.sleep(10)
                self.pointing(0.0)
                rospy.sleep(3)
                
                if self.unoccupied_seat_count != 0:
                    self.voice_logger("Please follow the arrow to take a empty seat. When you're ready to sit, please say 'thank you' to me to proceed.")
                    self.pointing(rot_arm)
                if self.unoccupied_seat_count == 0:
                    self.voice_logger("I am so sorry, all sit is fully occupied, When you're ok with this, please say 'thank you' to me to proceed")

                rot_arm = rot_arm + 0.1
                self.offering_chair = True
                self.reached_goal = False
                self.check_pos = False

            elif self.offering_chair and "thank you" in self.last_speech_input and not self.thank_you_received:
                self.voice_logger("You're welcome! Have a nice day.")
                rospy.sleep(6)
                self.turn(145, "right")
                rospy.sleep(5)
                self.move_forward(0.1)
                self.thank_you_received = True
                
                # origin_point = Point(*self.waypoints['origin'])
                # self.publish_coordinates(origin_point)
                rospy.sleep(5)
                
                self.offering_chair = False
                self.guest_count += 1
                self.current_guest = None
                self.current_favorite_drink = None
                self.start = False

            elif not self.there_is_human:
                self.head_shake()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Manager()
        node.run()
    except rospy.ROSInterruptException:
        pass
