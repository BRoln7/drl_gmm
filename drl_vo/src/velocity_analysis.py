#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OdomSubscriber:
    def __init__(self):
        self.odom_angular_z = 0.0
        self.drl_cmd_vel_angular_z = 0.0

        rospy.init_node('odom_subscriber', anonymous=True)

        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odomCallback)
        self.cmdvel_sub = rospy.Subscriber('/yocs_cmd_vel_mux/cmd_vel', Twist, self.cmdVelCallback)

    def odomCallback(self, msg):
        self.odom_angular_z = msg.twist.twist.linear.x
        # self.saveData()

    def cmdVelCallback(self, msg):
        self.drl_cmd_vel_angular_z = msg.linear.x
        self.saveData()

    def saveData(self):
        with open('real_velocity_data2.txt', 'a') as file:
            file.write('{:.2f}\n'.format(
                self.drl_cmd_vel_angular_z))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        odom_subscriber = OdomSubscriber()
        odom_subscriber.run()
    except rospy.ROSInterruptException:
        pass
