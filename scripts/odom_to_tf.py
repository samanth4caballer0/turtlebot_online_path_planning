#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry


# This node is used to publish tf based on the odometry of the vehicle
# as the Girona1000 publishes the tf through the driver of the INS
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
        "/turtlebot/kobuki/base_link",
        "/world_ned",
    )


def listener():
    rospy.init_node("odom_to_tf", anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
