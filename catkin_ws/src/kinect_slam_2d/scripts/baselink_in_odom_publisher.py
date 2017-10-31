#!/usr/bin/env python  
import roslib
import rospy
import tf

# send tf of the ODOM in the BASE_LINK frame
if __name__ == '__main__':
    rospy.init_node('baselink_in_odom_publisher')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((1.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        rate.sleep()