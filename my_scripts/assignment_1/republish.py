#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_pose():
    rospy.init_node('tf_to_pose_publisher', anonymous=True)
    
    source_frame = rospy.get_param('~source_frame', 'world')
    target_frame = rospy.get_param('~target_frame', 'end_effector_link')
    pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
            
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = target_frame
            
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]
            
            pose_pub.publish(pose_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass

