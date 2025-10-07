#!/usr/bin/env python3

import rospy
import actionlib
import math
from std_msgs.msg import Bool, String, Float64, UInt16MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose
import os
from datetime import datetime


class Manager:
    def __init__(self):
        rospy.init_node('interaction_manager', anonymous=True)

        # Subscribers
        self.human_state_sub = rospy.Subscriber("manager/human_state", Bool, self.human_state_callback) # human_range_finder
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

        self.humandown_sub = rospy.Subscriber(
            '/filtered_human_centroids', 
            PoseStamped, 
            self.human_down_callback
        )

        self.cmd_pub = rospy.Publisher(
            '/cmd_vel',
            Twist,
            queue_size=10)

        self.angle_sub = rospy.Subscriber("/theta_result", Float64, self.angle_callback)
        self.human_angle_sub = rospy.Subscriber("/human_result", Float64, self.human_angle_callback)


        self.arm1_joint_pub = rospy.Publisher(
            "/arm1_joint/command",
            Float64,
            queue_size=10
        )
        self.arm2_joint_pub = rospy.Publisher(
            "/arm2_joint/command",
            Float64,
            queue_size=10
        )
        self.arm3_joint_pub = rospy.Publisher(
            "/arm3_joint/command",
            Float64,
            queue_size=10
        )
        self.arm4_joint_pub = rospy.Publisher(
            "/arm4_joint/command",
            Float64,
            queue_size=10
        )

        self.gripper_joint_pub = rospy.Publisher(
            "/gripper_joint/command",
            Float64,
            queue_size=10
        )

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal",
            PoseStamped,
            queue_size=10
        )

        self.goal_status_sub = rospy.Subscriber(
            '/move_base/status',
            GoalStatusArray,
            self.goal_status_callback)

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
        self.unoccupied_seat_count = 0 
        self.robot_odom = (0, 0)
        self.laying = False
        self.goal_reached = False
        self.currnt_time = None

        self.last_speech_input = ""

        self.guest_name = ["julia", "emma", "sara", "laura", "susan", "john", "lucas", "william", "kevin", "peter", "robin", "jeffery", "tan", "sarah", "adam"]

        self.guest_drink = ["hot chocolate", "coke", "soy", "water", "lime juice", "tea", "coffee", "soda", "milk", "latte"]

        self.unoccupied_seats = []
        self.guest_inside = [0,0,0,0,0]
        self.drink_inside = [0,0,0,0,0]
        self.rate = rospy.Rate(10)

        self.luggage_dimentions = [0.10, 0.28, 0.37]
        self.gripper_range = [-0.3988, 0.6]

        # Navigation client
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Waypoints
        self.waypoints = {
            # 'host_location': (1.3, 1.51, math.pi * 1),  # Host's location
            'kitchen': (0.761,2.0, math.pi * 1),  # Host's location
            'door': (1.09, 1.61, math.pi * 2)  # Origin location
        }

    def publish_coordinates(self, target_point):
        self.reached_goal = False
        rospy.loginfo(f"Moving to target coordinates: x={target_point.x}, y={target_point.y}, z={target_point.z}")
        self.move_to_goal(target_point.x, target_point.y, target_point.z)
        self.reached_goal = True

    def odom_callback(self, msg):
        self.robot_odom = msg.pose
        self.robot_pose = msg.pose.pose
        self.robot_orientation = msg.pose.pose.orientation
        _, _, self.current_theta = euler_from_quaternion(
            [self.robot_orientation.x, self.robot_orientation.y, self.robot_orientation.z, self.robot_orientation.w]
        )


    def human_down_callback(self, msg):
        if msg.pose.orientation.w != 0.0:
            self.laying = True
            self.laying_pose = msg.pose

        self.there_is_human = True

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

        quaternion = quaternion_from_euler(0.0, 0.0, math.radians(theta))
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(*quaternion))

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def pointing(self, angle):
        msg = Float64()
        msg.data = angle
        self.arm1_joint_pub.publish(msg)

    
    def angle_callback(self, msg):
        self.angle_res = msg.data

    def human_angle_callback(self, msg):
        self.human_angle_res = msg.data
        
    def pointing_chair(self):
        if self.angle_res >= 0.85:
            real_angle = self.angle_res - 0.85
            self.pointing(real_angle)
        # if self.angle_res < 0.85:
        #     real_angle = 1.25
        #     self.pointing(real_angle)

    def pointing_host(self):
        if self.human_angle_res >= 0.85:
            human_real_angle = self.human_angle_res - 0.85
            self.pointing(human_real_angle)
        # elif self.human_angle_res < 0.85:
        #     human_real_angle = self.human_angle_res + 0.85
        #     self.pointing(human_real_angle)

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


    def rotate_towards(self, x_target, y_target):
    
        current_pose = self.robot_pose
        if current_pose is None:
            rospy.logwarn("Current pose not available")
            return

        # Get the current position and orientation
        x_current = current_pose.position.x
        y_current = current_pose.position.y
        orientation = current_pose.orientation

        # Convert quaternion orientation to Euler angles
        _, _, theta_current = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Compute the desired angle
        dx = x_target - x_current
        dy = y_target - y_current
        theta_target = math.atan2(dy, dx)

        # Compute the difference
        delta_theta = theta_target - theta_current

        # Normalize the angle to be within -pi to pi
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        # Create a Twist message to rotate the robot
        cmd_msg = Twist()
        cmd_msg.angular.z = 0.5 * - delta_theta  # Adjust the rotation speed (0.5 is an example)

        # Publish the Twist message
        self.cmd_pub.publish(cmd_msg)

    
    def arm_basic_controller(self, state):
        msg = Float64()
        if state == 0:
            msg.data = 0.0
            self.arm1_joint_pub.publish(msg)

            msg.data = -1.4
            self.arm2_joint_pub.publish(msg)

            msg.data = 2.2
            self.arm3_joint_pub.publish(msg)

            msg.data = 0.3
            self.arm4_joint_pub.publish(msg)

        elif state == 1:
            msg.data = 0.0
            self.arm1_joint_pub.publish(msg)

            msg.data = -0.6
            self.arm2_joint_pub.publish(msg)

            msg.data = 1.0
            self.arm3_joint_pub.publish(msg)

            msg.data = 0.6
            self.arm4_joint_pub.publish(msg)

            


        elif state == -1:
            msg.data = 0.0
            self.arm1_joint_pub.publish(msg)

            msg.data = 0.0
            self.arm2_joint_pub.publish(msg)

            msg.data = 0.0
            self.arm3_joint_pub.publish(msg)

            msg.data = 0.0
            self.arm4_joint_pub.publish(msg)

        elif state == 2:
            msg.data = -0.6
            self.arm4_joint_pub.publish(msg)

        elif state ==3:
            msg.data = 0.0
            self.arm1_joint_pub.publish(msg)

            msg.data = 1.18
            self.arm2_joint_pub.publish(msg)

            msg.data = 1.69
            self.arm3_joint_pub.publish(msg)

            msg.data = -1.65
            self.arm4_joint_pub.publish(msg)

        
        elif state == 4:
            msg.data = 0.6
            self.arm4_joint_pub.publish(msg)

        
        elif state == 5:
            msg.data = 1.4
            self.arm1_joint_pub.publish(msg)
        
        elif state == 6:
            msg.data = 0.0
            self.arm1_joint_pub.publish(msg)



    def gripper_controller(self, angle):
        msg = Float64()
        msg.data = angle
        self.gripper_joint_pub.publish(msg)
            

    def play_sound(self, file_name):
        os.system(f"mpg321 {'/home/mustar/catkin_ws/src/robocup/receptionist/scripts/final_task/'+file_name}")


    def move2goal(self, location, angle, wait=True):
        self.goal_reached = False
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = location[0]
        msg.pose.position.y = location[1]

        quaternion = quaternion_from_euler(0.0, 0.0, math.radians(angle))

        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

        while wait:
            if self.goal_reached:
                self.navigate = True
                break
            self.rate.sleep()
    
    def goal_status_callback(self, msg):
        
        for status in msg.status_list:
            if status.status == 3:
                self.goal_reached = True
            elif status.status == 4:
                self.goal_reached = False

    def run(self):
        
        self.arm_basic_controller(state=0)
        self.gripper_controller(self.gripper_range[1])

        #move 2 kitchen
        self.goal_reached = False
        self.move2goal((-3.66, 2.46), 90.0, wait=True)
        
        while not rospy.is_shutdown():
            if self.laying:
                self.fall_time = datetime.now().strftime('%H:%M')
                for i in range(2):
                    self.voice_logger("are you ok?")
                    time.sleep(3)
                    self.play_sound("beep.mp3")
                break
            self.rate.sleep()
        
        self.voice_logger("Calling Emergency Service")

        rospy.sleep(3)

        self.play_sound("sound_1.mp3")

        

        # move 2 door
        self.goal_reached = False
        self.move2goal((0.375, 1.59), 0.0, wait=True)

        self.there_is_human = False


        # start receptinest 

        while not rospy.is_shutdown():

            if self.there_is_human:
                print("yes")
                self.voice_logger("Human detected infront of the door")
                time.sleep(1)
                self.voice_logger("Hello, who are you?")

                while not rospy.is_shutdown():
                    rospy.sleep(1)
                    if "doctor" in self.last_speech_input:
                    
                        self.voice_logger(f"Thank you for coming, let me carry your bag")
                        
                        rospy.sleep(2)

                        self.voice_logger(f"please hang the bag on my mouth")

                        rospy.sleep(1)

                        self.arm_basic_controller(state=1)
                        

                        rospy.sleep(1)

                        self.gripper_controller(self.gripper_range[0])

                        self.voice_logger(f"once you hang the bag, please say Done")

                        while not rospy.is_shutdown():
                            rospy.sleep(1)
                            if self.last_speech_input == "done":
                                self.gripper_controller(self.gripper_range[1])
                                self.voice_logger(f"Thank you")
                                break
                            self.rate.sleep()
                        break
                    self.rate.sleep()

                self.voice_logger(f"Please follow me to my host")

                # # navigation to kitchen
                self.goal_reached = False
                
                self.move2goal((-3.66, 2.46), 90.0, wait=True)
                rospy.sleep(25)


                self.voice_logger(f"Doctor, please stand by my left side")

                self.move_forward(0.2)

                # drop the luggage
                
                self.arm_basic_controller(state=3)
                rospy.sleep(5)
                self.gripper_controller(self.gripper_range[0])
                rospy.sleep(1)

                # slowly move back
                self.move_forward(-0.20)

                self.arm_basic_controller(state=0)
                self.gripper_controller(self.gripper_range[1])

                rospy.sleep(3)
                # talk to doctor
                self.arm_basic_controller(state=5)
                rospy.sleep(2)
                self.voice_logger(f"Doctor,this is my host, he fel down at {self.fall_time} this morning")
                rospy.sleep(2)
                self.arm_basic_controller(state=6)
                self.voice_logger(f"i tried to interact with him, but he is not responding.")



                break
            self.rate.sleep()

            

if __name__ == '__main__':
    try:
        node = Manager()
        node.run()
    except rospy.ROSInterruptException:
        pass

