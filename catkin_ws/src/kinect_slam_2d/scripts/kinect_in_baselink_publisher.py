#!/usr/bin/env python  
import roslib
import rospy
import tf

# send tf of the KINECT in the BASE_LINK coordinate frame
if __name__ == '__main__':
    rospy.init_node('kinect_in_baselink_publisher')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.1, 0.0, 0.1),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "kinect",
                         "base_link")
        rate.sleep()

# To visualize the TF tree:
# rosrun rqt_tf_tree rqt_tf_tree