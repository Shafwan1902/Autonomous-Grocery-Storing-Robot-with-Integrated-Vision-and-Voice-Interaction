#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs  # Make sure to import tf2_geometry_msgs

def pose_callback(point_msg):
    
    # rospy.loginfo("Original PointStamped: position (x: %.2f, y: %.2f, z: %.2f) in frame '%s'",
    #           point_msg.point.x,
    #           point_msg.point.y,
    #           point_msg.point.z,
    #           point_msg.header.frame_id)

    # Transform the pose message from the source frame to the target frame
    try:
        # Lookup the transform from source to target frame
        transform = tf_buffer.lookup_transform(target_frame, point_msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        # Create a PoseStamped from the PointStamped
        pose_msg = geometry_msgs.msg.PoseStamped()
        pose_msg.header = point_msg.header
        pose_msg.pose.position = point_msg.point
        pose_msg.pose.orientation.w = 1.0  # Assuming no orientation for PointStamped, set a default

        # Transform the pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_msg, transform)

         # Log the transformed pose result
        # rospy.loginfo("Transformed Pose: position (x: %.2f, y: %.2f, z: %.2f)", # orientation (x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
        #               transformed_pose.pose.position.x,
        #               transformed_pose.pose.position.y,
        #               transformed_pose.pose.position.z)
                    #   transformed_pose.pose.orientation.x,
                    #   transformed_pose.pose.orientation.y,
                    #   transformed_pose.pose.orientation.z,
                    #   transformed_pose.pose.orientation.w)
        
        # Publish the transformed pose
        transformed_pose.header.frame_id = target_frame
        transformed_pose.header.stamp = rospy.Time.now()
        pose_publisher.publish(transformed_pose)

        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("Transform error: %s", str(e))

def main():
    global tf_buffer
    global target_frame
    global pose_publisher

    # Initialize the ROS node
    rospy.init_node('frame_transformer')

    # Get parameters
    # source_frame = "camera_depth_frame"
    target_frame = "map"

    # Create a TransformListener and Buffer
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Create a Publisher for the transformed pose
    pose_publisher = rospy.Publisher(
        '/human_centroids/map', 
        geometry_msgs.msg.PoseStamped, 
        queue_size=1
    )

    # Subscribe to the PoseStamped topic
    rospy.Subscriber(
        'human_centroids', 
        geometry_msgs.msg.PointStamped, 
        pose_callback
    )

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
