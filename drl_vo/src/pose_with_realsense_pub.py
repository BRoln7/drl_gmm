#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo

class PosePublisher:
    def __init__(self):
        rospy.init_node('pose_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)

    def camera_info_callback(self, msg):
        camera_timestamp = msg.header.stamp
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            pose_msg = PoseStamped()
            pose_msg.header.stamp = camera_timestamp
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            self.pose_pub.publish(pose_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform between 'map' and 'base_footprint'.")

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
