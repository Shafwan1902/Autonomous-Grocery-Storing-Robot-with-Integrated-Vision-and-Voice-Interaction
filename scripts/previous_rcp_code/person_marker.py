#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from dynamixel_msgs.msg import JointState
import tf

class PersonMarkerNode:

    def __init__(self):
        # Subscriber to the filtered positions
        self.centroid_sub = rospy.Subscriber(
            '/filtered_human_centroids', 
            PoseStamped, 
            self.centroid_callback
        )

        # Subscriber to the head joint state
        self.head_state_sub = rospy.Subscriber(
            "/head_joint/state",
            JointState,
            self.head_state_callback
        )

        # Publisher for markers
        self.marker_pub = rospy.Publisher(
            "/human_visualization",
            Marker,
            queue_size=10
        )

        self.markers = {}  # Dictionary to store markers by ID
        self.head_joint_state = 0.0  # Placeholder for head joint state
        self.current_ids = set()  # Track currently detected IDs

    def centroid_callback(self, msg):
        person_id = msg.header.seq  # Assuming unique ID for each person

        # Visualize each detected person
        self.visualization(person_id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

        # Track detected IDs
        self.current_ids.add(person_id)

        # Handle marker deletion after all detections have been processed
        rospy.Timer(rospy.Duration(0.1), self.remove_undetected_markers, oneshot=True)

    def head_state_callback(self, msg):
        self.head_joint_state = msg.current_pos

    def visualization(self, person_id, x, y, z):
        body_marker = Marker()

        body_marker.header.frame_id = 'map'  # Set to "map" frame
        body_marker.header.stamp = rospy.Time.now()
        body_marker.ns = 'body'
        body_marker.id = person_id  # Use object ID as marker ID
        body_marker.type = Marker.CUBE
        body_marker.scale.x = 0.5
        body_marker.scale.y = 0.5
        body_marker.scale.z = 2.0
        body_marker.pose.position.x = x
        body_marker.pose.position.y = y
        body_marker.pose.position.z = z

        # Calculate orientation based on head joint state
        quaternion = tf.transformations.quaternion_from_euler(0.0, -self.head_joint_state, 0.0)

        body_marker.pose.orientation.x = quaternion[0]
        body_marker.pose.orientation.y = quaternion[1]
        body_marker.pose.orientation.z = quaternion[2]
        body_marker.pose.orientation.w = quaternion[3]

        body_marker.color.r = 0.0
        body_marker.color.g = 1.0
        body_marker.color.b = 1.0
        body_marker.color.a = 0.5

        # Set marker lifetime to 1 second
        body_marker.lifetime = rospy.Duration(5.0)

        self.marker_pub.publish(body_marker)
        self.markers[person_id] = body_marker  # Update markers dictionary

    def remove_undetected_markers(self, event):
        # Find markers that need to be deleted
        detected_ids = set(self.markers.keys())
        to_delete = detected_ids - self.current_ids

        for marker_id in to_delete:
            self.delete_marker(marker_id)

        # Reset the current_ids set for the next round of detections
        self.current_ids.clear()

    def delete_marker(self, marker_id):
        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.ns = 'body'
        delete_marker.id = marker_id
        delete_marker.action = Marker.DELETE

        self.marker_pub.publish(delete_marker)
        del self.markers[marker_id]

if __name__ == "__main__":
    rospy.init_node('person_marker')
    PersonMarkerNode()
    rospy.spin()
