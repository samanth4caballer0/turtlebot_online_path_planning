#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

base_link = "base_link"
fixed_frame = "odom"


# This node is used to publish tf based on the odometry of the vehicle
def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ),
        rospy.Time.now(),
        base_link,
        fixed_frame,
    )


if __name__ == "__main__":
    rospy.init_node("odom_to_tf", anonymous=True)
    base_link = rospy.get_param("~base_link")
    fixed_frame = rospy.get_param("~fixed_frame")
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()
