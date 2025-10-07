#!/usr/bin/env python3

import rospy
import actionlib
import math
from std_msgs.msg import Bool, String, Float64, UInt16MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

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

        # State Variables
        self.there_is_human = False
        self.voice_log = True  # Enables/disables voice logging (used to control if the robot should speak)
        self.start = False
        self.intro = False
        self.current_guest = None
        self.current_favorite_drink = None
        self.current_host = "John"
        self.host_favorite_drink = "coffee"
        self.guest_count = 0  
        self.waiting_for_name = False
        self.waiting_for_drink = False
        self.waiting_for_yes = False
        self.reached_goal = False
        self.offering_chair = False
        self.thank_you_received = False
        self.unoccupied_seat_count = 0  # Track the number of unoccupied seats

        # New instance variable to store last speech input
        self.last_speech_input = ""

        self.guest_data = {
            "Julia": "Hot Chocolate",
            "Emma": "Coke",
            "Julia": "Soy",
            "Sara ": "Water",
            "Susan ": "Lime juice",
            "John": "Tea",
            "Lucas": "Coffee",
            "William ": "Soda",
            "Kevin ": "Milk",
            "Peter ": "Latte",
            "Robin": "Hot Chocolate",
            "Jeffery": "Coke",
            "Tan": "Soy",
            "Sarah ": "Water",
            "Adam ": "Lime juice",
        }

        self.unoccupied_seats = []
        self.rate = rospy.Rate(10)

        # Navigation client
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        # Waypoints
        self.waypoints = {
            'host_location': (0.771, -0.03, math.pi *2),  # Host's location
            'origin': (-0.0501, -0.0901, math.pi *2)  # Origin location
        }

    def publish_coordinates(self, target_point):
        self.reached_goal = False
        rospy.loginfo(f"Moving to the target coordinates: x={target_point.x}, y={target_point.y}, z={target_point.z}")
        self.move_to_goal(target_point.x, target_point.y, target_point.z)
        self.reached_goal = True

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
        rospy.loginfo(f"Received speech input: {text}")  # Debugging log
        self.last_speech_input = text  # Ensure this is updated every time

        if not self.there_is_human:
            return

        if self.start and not self.intro:
            self.head_controller(0.0)
            self.voice_logger("Hello, nice to meet you! May I know your name?")
            self.intro = True
            self.waiting_for_name = True
            return

        if self.waiting_for_name and not self.current_guest:
            if text in ["Sarah","alex","siri","James","Emma","Lily","Tom","Ben","Ava","Sam"]:
                self.current_guest = text
                self.voice_logger(f"Nice to meet you, {self.current_guest}. What is your favorite drink?")
                self.waiting_for_name = False
                self.waiting_for_drink = True
            else:
                self.voice_logger("Sorry, I didn't recognize that name. Please try again.")
            return



        if self.waiting_for_drink and not self.current_favorite_drink:
            if text in ["coffee", "juice", "tea","Water", "Soda", "Lemonade","Cola", "milk", "Mocha","tea"]:
                self.current_favorite_drink = text
                self.guest_data[self.current_guest] = self.current_favorite_drink
                self.voice_logger(f"Thank you for the information. Please say 'yes' so I can guide you.")
                self.waiting_for_drink = False
                self.waiting_for_yes = True
            else:
                self.voice_logger("Sorry, I didn't recognize that drink. Please try again.")
            return

        if self.waiting_for_yes and "yes" in text:
            self.voice_logger("Please follow me to my host")
            target_point = Point(*self.waypoints['host_location'])
            self.publish_coordinates(target_point)
            self.waiting_for_yes = False
            rospy.sleep(20)

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

        self.unoccupied_seat_count = num_chairs  # Update the count of unoccupied seats
        self.there_is_human = num_humans > 0

    def human_state_callback(self, msg):
        self.there_is_human = msg.data

    def head_shake(self):
        self.head_controller(0.0)
        rospy.sleep(0.5)
        self.head_controller(0.1)
        rospy.sleep(2)

    def move_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(*quaternion))

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

    def run(self):
        self.head_controller(0.0)
        
        while not rospy.is_shutdown():

            if self.there_is_human and self.guest_count < 3:
                if not self.start:
                    rospy.loginfo("Human detected, waiting 5 seconds before greeting...")
                    rospy.sleep(5)
                    self.start = True
                    self.head_controller(0.0)  # Keep the camera still when a human is detected

            elif self.reached_goal and not self.offering_chair:
                seat_message = f"There are {self.unoccupied_seat_count} seats available."
                rospy.sleep(5)
                self.reached_goal = False  # Reset goal flag to prevent repeated triggering
                self.voice_logger(f"Hi my host, you have a visitor named {self.current_guest}. For your information, their favorite drink is {self.current_favorite_drink}.")
                self.voice_logger(f"{self.current_guest}, this is {self.current_host}, my host. {self.current_host}'s favorite drink is {self.host_favorite_drink}.")
                self.voice_logger(f"{seat_message} Please feel free to take a seat. When you're ready, please say 'thank you' to me for me to return to the original position.")
                
                self.offering_chair = True  # Mark that the chair is being offered and we're waiting for "thank you"
                self.thank_you_received = False  # Reset the thank you flag

            elif self.offering_chair:
                # Continuously check if "thank you" has been received
                if "thank you" in self.last_speech_input:
                    self.voice_logger("You're welcome! Have a nice day.")
                    self.thank_you_received = True
                    
                    origin_point = Point(*self.waypoints['origin'])
                    self.publish_coordinates(origin_point)
                    
                    self.offering_chair = False
                    self.guest_count += 1  # Increment guest count
                    self.current_guest = None
                    self.current_favorite_drink = None
                    self.start = False  # Reset start for the next guest
                    self.intro = False  # Reset intro for the next guest
                    self.there_is_human = False  # Reset human detection for the next guest

            elif not self.there_is_human:
                # Move the camera as no human is detected
                self.head_shake()

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Manager()
        node.run()
    except rospy.ROSInterruptException:
        pass
