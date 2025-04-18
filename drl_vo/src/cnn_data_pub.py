#!/usr/bin/env python
#!/usr/bin/env python
#
# revision history: xzt
#  20210604 (TE): first version
#
# usage:
#
# This script is to publish pedestrian kinematic maps and lidar historical map.
#------------------------------------------------------------------------------
import numpy as np
import rospy
import tf
from cnn_msgs.msg import CNN_data
# custom define messages:
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from tf.transformations import quaternion_matrix, translation_matrix, concatenate_matrices
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from sensor_msgs.msg import LaserScan

# parameters:
NUM_TP = 10     # the number of timestamps
NUM_PEDS = 34+1 # the number of total pedestrians

class CnnData:
    # Constructor
    def __init__(self):
        # initialize data:  
        self.ped_pos_map = []
        self.scan = [] #np.zeros(720)
        self.scan_all = np.zeros(1080)
        self.goal_cart = np.zeros(2)
        self.goal_final_cart = np.zeros(2)
        self.vel = np.zeros(2)

        # temporal data:
        self.ped_pos_map_tmp = np.zeros((2,80,80))  # cartesian velocity map
        self.scan_tmp = np.zeros(720)
        self.scan_all_tmp = np.zeros(1080)

        # initialize ROS objects
        self.listener = tf.TransformListener()
        self.ped_sub = rospy.Subscriber("/track_ped", TrackedPersons, self.ped_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.laser_pub = rospy.Publisher("/laser", LaserScan, queue_size=1, latch=False)
        # self.goal_sub = rospy.Subscriber("/cnn_goal", Point, self.goal_callback)
        self.goal_sub = rospy.Subscriber("/cnn_goal", Point, self.goal_callback)
        self.vel_sub = rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.vel_callback)
        self.cnn_data_pub = rospy.Publisher('/cnn_data', CNN_data, queue_size=1, latch=False)
      
        # timer:
        self.rate = 20  # 20 Hz velocity controller
        self.ts_cnt = 0  # maximum 10 timesteps
        # initialize timer for controller update
        self.timer = rospy.Timer(rospy.Duration(1./self.rate), self.timer_callback)
        
    # Callback function for the pedestrian subscriber
    def ped_callback(self, trackPed_msg):
        # get the pedstrain's position:
        self.ped_pos_map_tmp = np.zeros((2,80,80))  # cartesian velocity map
        if(len(trackPed_msg.tracks) != 0):  # tracker results
            for ped in trackPed_msg.tracks:
                #ped_id = ped.track_id 
                # create pedestrian's postion costmap: 10*10 m
                x = ped.pose.pose.position.x
                y = ped.pose.pose.position.y
                vx = ped.twist.twist.linear.x
                vy = ped.twist.twist.linear.y
                # 20m * 20m occupancy map:
                if(x >= 0 and x <= 20 and np.abs(y) <= 10):
                    # bin size: 0.25 m
                    c = int(np.floor(-(y-10)/0.25))
                    r = int(np.floor(x/0.25))

                    if(r == 80):
                        r = r - 1
                    if(c == 80):
                        c = c - 1
                    # cartesian velocity map
                    self.ped_pos_map_tmp[0,r,c] = vx
                    self.ped_pos_map_tmp[1,r,c] = vy

    # Callback function for the scan measurement subscriber
    def scan_callback(self, laserScan_msg):
        # get the laser scan data:
        try:
          (trans, rot) = self.listener.lookupTransform('base_link', 'sensor_ray', rospy.Time(0))
          # Construct transformation matrix
          trans_matrix = translation_matrix(trans)
          rot_matrix = quaternion_matrix(rot)
          transformation_matrix = concatenate_matrices(trans_matrix, rot_matrix)
          # Transform each point in the laser scan
          transformed_ranges = []
          for range_val, angle in zip(laserScan_msg.ranges, np.arange(laserScan_msg.angle_min,\
            laserScan_msg.angle_max, laserScan_msg.angle_increment)):
              if np.isinf(range_val):
                  transformed_ranges.append(range_val)
              else:
                  point = [range_val * np.cos(angle), range_val * np.sin(angle), 0, 1.0]  # Append 1 to make it homogeneous
                  transformed_point = np.dot(transformation_matrix, point)[:3]  # Apply transformation
                  transformed_ranges.append(np.linalg.norm(transformed_point))

          transformed_scan_msg = LaserScan()
          transformed_scan_msg.header.frame_id = "base_link"
          transformed_scan_msg.header.stamp = rospy.Time.now()
          transformed_scan_msg.angle_increment = laserScan_msg.angle_increment
          transformed_scan_msg.time_increment = laserScan_msg.time_increment
          transformed_scan_msg.scan_time = laserScan_msg.scan_time
          transformed_scan_msg.angle_min = laserScan_msg.angle_min + (1/3)*laserScan_msg.angle_max
          transformed_scan_msg.angle_max = laserScan_msg.angle_max - (1/3)*laserScan_msg.angle_max
          transformed_scan_msg.ranges = transformed_ranges[180:900]
          self.laser_pub.publish(transformed_scan_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transform or transform laser scan.")
        
        self.scan_tmp = np.zeros(720)
        self.scan_all_tmp = np.zeros(1080)
        scan_data = np.array(laserScan_msg.ranges, dtype=np.float32)
        scan_data[np.isnan(scan_data)] = 0.
        scan_data[np.isinf(scan_data)] = 0.
        self.scan_tmp = scan_data[180:900]
        self.scan_all_tmp = scan_data

    # Callback function for the current goal subscriber
    def goal_callback(self, goal_msg):
        # Cartesian coordinate:
        self.goal_cart = np.zeros(2)
        self.goal_cart[0] = goal_msg.x
        self.goal_cart[1] = goal_msg.y

    # Callback function for the velocity subscriber
    def vel_callback(self, vel_msg):
        self.vel = np.zeros(2)
        self.vel[0] = vel_msg.linear.x
        self.vel[1] = vel_msg.angular.z
        
     # function that runs every time the timer finishes to ensure that velocity commands are sent regularly
    def timer_callback(self, event):  
        # generate the trajectory of pedstrians:
        self.ped_pos_map = self.ped_pos_map_tmp
        self.scan.append(self.scan_tmp.tolist())
        self.scan_all = self.scan_all_tmp

        self.ts_cnt = self.ts_cnt + 1
        if(self.ts_cnt == NUM_TP): 
            # publish cnn data:
            cnn_data = CNN_data()
            cnn_data.ped_pos_map = [float(val) for sublist in self.ped_pos_map for subb in sublist for val in subb]
            #cnn_data.ped_pos_map1 = [float(val) for sublist in self.ped_pos_map1 for subb in sublist for val in subb]
            cnn_data.scan = [float(val) for sublist in self.scan for val in sublist]
            cnn_data.scan_all = self.scan_all 
            cnn_data.depth = [] #[float(val) for sublist in self.depth for val in sublist]
            cnn_data.image_gray = [] #[float(val) for sublist in self.image_gray for val in sublist]
            cnn_data.goal_cart = self.goal_cart
            cnn_data.goal_final_polar = []
            cnn_data.vel = self.vel
            self.cnn_data_pub.publish(cnn_data)

            # reset the position data list:
            # pop the oldest scan
            self.ts_cnt = NUM_TP-1
            self.scan = self.scan[1:NUM_TP]

if __name__ == '__main__':
    try:
        rospy.init_node('cnn_data')
        CnnData()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
