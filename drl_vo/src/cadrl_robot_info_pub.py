#!/usr/bin/env python
#
# revision history: hill
#  20240309: first version
#
# usage:
#
# This script is to publish the robot pose and velocity.
#------------------------------------------------------------------------------

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Pose, Vector3
import tf
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import numpy as np

class RobotInfo():
    def __init__(self):
      #self.tf_listener = tf.TransformListener()
      self.robot_odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
      self.robot_velocity_pub = rospy.Publisher('~velocity', Vector3, queue_size=10)
      #self.robot_pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=10)
      #self.rate = rospy.Rate(30)
      #self.pose = PoseStamped()

    def odom_callback(self, odom_msgs):
       velocity = Vector3()
       velocity.x = odom_msgs.twist.twist.linear.x
       velocity.y = odom_msgs.twist.twist.linear.y
       velocity.z = 0
       self.robot_velocity_pub.publish(velocity)

    

if __name__ == '__main__':
    try:
        rospy.init_node('robot_info')
        tp = RobotInfo()
        # spin() simply keeps python from exiting until this node is stopped    
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

       

