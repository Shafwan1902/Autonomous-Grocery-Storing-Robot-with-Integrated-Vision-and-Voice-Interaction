#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math

class BaseRotatorToChair:
    def __init__(self):
        rospy.init_node("rcp_pointing_node", anonymous=True)

        self.base_pub = rospy.Publisher("/arm1_joint/command", 
                                        Float64, 
                                        queue_size=10)
        
        rospy.Subscriber("/rcp/3d_depth/object_position",
                          Point, 
                          self.point_callback)

        # rospy.loginfo("✅ BaseRotatorToChair node started.")

    def point_callback(self, msg):
        x, z = msg.x, msg.z
        if z == 0 or math.isnan(x) or math.isnan(z):
            rospy.logwarn("❌ Invalid point received.")
            return

        theta = math.atan2(-x, z)
        theta = max(min(theta, math.radians(90)), math.radians(-90))  # Clamp

        self.base_pub.publish(Float64(theta))
        rospy.loginfo(f"🎯 Pointing → θ={math.degrees(theta):.2f}°")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        BaseRotatorToChair().run()
    except rospy.ROSInterruptException:
        pass
