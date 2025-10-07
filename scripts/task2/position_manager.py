#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class PositionManager:
    def __init__(self):
        rospy.init_node('position_manager', anonymous=True)

        # Subscribers
        rospy.Subscriber("/save_position", PoseStamped, self.save_position_callback)
        rospy.Subscriber("/get_position", String, self.get_position_callback)

        # Publishers
        self.position_pub = rospy.Publisher("/return_position", PoseStamped, queue_size=10)

        # Variables to store positions
        self.saved_host_position = None
        self.saved_guest1_position = None
        self.saved_guest2_position = None
        self.saved_guest3_position = None

    def save_position_callback(self, msg):
        # The message should contain the position and a keyword in the frame_id to indicate which position to save
        if msg.header.frame_id == "host":
            self.saved_host_position = msg
            rospy.loginfo("Host position saved.")
        elif msg.header.frame_id == "guest1":
            self.saved_guest1_position = msg
            rospy.loginfo("Guest 1 position saved.")
        elif msg.header.frame_id == "guest2":
            self.saved_guest2_position = msg
            rospy.loginfo("Guest 2 position saved.")
        elif msg.header.frame_id == "guest3":
            self.saved_guest3_position = msg
            rospy.loginfo("Guest 3 position saved.")
        else:
            rospy.logwarn("Unknown position type in frame_id: %s", msg.header.frame_id)

    def get_position_callback(self, msg):
        # The message should contain the keyword of the position to retrieve
        if msg.data == "host" and self.saved_host_position is not None:
            self.position_pub.publish(self.saved_host_position)
            rospy.loginfo("Returning host position.")
        elif msg.data == "guest1" and self.saved_guest1_position is not None:
            self.position_pub.publish(self.saved_guest1_position)
            rospy.loginfo("Returning guest 1 position.")
        elif msg.data == "guest2" and self.saved_guest2_position is not None:
            self.position_pub.publish(self.saved_guest2_position)
            rospy.loginfo("Returning guest 2 position.")
        elif msg.data == "guest3" and self.saved_guest3_position is not None:
            self.position_pub.publish(self.saved_guest3_position)
            rospy.loginfo("Returning guest 3 position.")
        else:
            rospy.logwarn("No position saved for: %s", msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PositionManager()
        node.run()
    except rospy.ROSInterruptException:
        pass
