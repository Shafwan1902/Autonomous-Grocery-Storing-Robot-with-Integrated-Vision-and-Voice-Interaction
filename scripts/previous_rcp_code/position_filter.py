#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

class PositionFilterNode:

    def __init__(self):
        # Subscriber to the PoseStamped in the "map" frame
        self.position_sub = rospy.Subscriber(
            '/human_centroids/map', 
            PoseStamped, 
            self.filter_position_callback
        )

        # Publisher for filtered positions
        self.filtered_pub = rospy.Publisher(
            '/filtered_human_centroids', 
            PoseStamped, 
            queue_size=10
        )

        # Define the area boundaries
        self.max_x = 1.67
        self.min_x = -5.13
        self.max_y = 4.31
        self.min_y = -2.32

        # Distance threshold to consider two points as the same object
        self.proximity_threshold = 0.5  # Adjust as necessary

        # Dictionary to store IDs and positions
        self.positions_dict = {}
        self.current_id = 0

    def filter_position_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Check if the position is within the defined area
        if self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y:
            # Assign or reuse an ID based on proximity
            obj_id = self.assign_id(x, y)
            
            # Store the ID in the frame_id (you can choose another method to attach the ID)
            msg.header.frame_id = str(obj_id)
            
            # Publish the filtered position with the assigned ID
            self.filtered_pub.publish(msg)
        # else:
            # rospy.loginfo(f"Position (x: %.2f, y: %.2f) in frame '%s'",
            #   x,
            #   y,
            #   msg.header.frame_id)

    def assign_id(self, x, y):
        for obj_id, position in self.positions_dict.items():
            if self.is_within_proximity(position, (x, y)):
                return obj_id
        
        # If no close position is found, assign a new ID
        self.current_id += 1
        self.positions_dict[self.current_id] = (x, y)
        return self.current_id

    def is_within_proximity(self, pos1, pos2):
        """Check if two positions are within the proximity threshold."""
        distance = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
        return distance <= self.proximity_threshold

if __name__ == "__main__":
    rospy.init_node('position_filter')
    node = PositionFilterNode()
    rospy.spin()
