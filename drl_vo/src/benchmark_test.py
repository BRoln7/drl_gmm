#!/usr/bin/env python

import rospy
import numpy as np
from numpy import random
import math
import time
from geometry_msgs.msg import PoseStamped, Point, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from pedsim_msgs.msg import TrackedPerson, TrackedPersons, TrackedGroup, TrackedGroups
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState

class nav_test:
  def __init__(self):
     self.ROBOT_RADIUS = 0.3
     self.NAV_TIME = 22 #35
     self.curr_pose = Pose()
     self.map = OccupancyGrid()
     self.goal_position = Point()
     self.groups = TrackedGroups()
     self.peds = TrackedPersons()

     self.nav_flag = False
     self.settle_num = 0
     self.intrusion_in_flag = False
     self.intrusion_out_flag = False
    # benchmark
     self.nav_nums = 0
     self.success_nums = 0
     
     self.collision_nums = 0
     self.collision_current = 0
     self.collision_rate = 0

     self.collision_ped_nums = 0
     self.collision_ped_current = 0
     self.collision_ped_rate = 0
     
     self.nav_times = 0
     self.time_start = 0
     self.time_end = 0
     
     self.intrusion_nums = 0
     self.intrusion_nums_current = 0
     self.intrusion_times = 0
     self.intrusion_times_current = 0
     self.intrusion_timer_start = 0
     self.intrusion_timer_end = 0
     self.intrusion_group_id = 0

     self.trajectory_length = 0
     self.trajectory_length_current = 0
     self.previous_position = Point()
     self.current_position = Point()

     self._map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
     self._goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
     self._robot_pos_sub = rospy.Subscriber("/robot_pose", PoseStamped, self._robot_pose_callback)
     self._final_goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, self._final_goal_callback)
     self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
     self._scan_sub = rospy.Subscriber("/laser", LaserScan, self._scan_callback)
     self._groups_sub = rospy.Subscriber('/track_group', TrackedGroups, self._group_callback)
     self._ped_sub = rospy.Subscriber("/track_ped", TrackedPersons, self._ped_callback)
     self._initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
     self._set_robot_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
     self._record_timer = rospy.Timer(rospy.Duration(0.02), self._timer_callback)
     self._intrude_timer = rospy.Timer(rospy.Duration(0.02), self._timer2_callback)

  # for goal_pub
  def _timer_callback(self, event):
      # publish new goal
      if (self.nav_flag == False):
          self._publish_random_goal()
          self.nav_nums += 1
          rospy.logwarn("number %d", self.nav_nums)
          if self.nav_nums > 25:
              self.nav_nums = 25
              rospy.logwarn("nav_nums:%d, sucess_rate:%.2f, collision_nums:%.2f, collsion_ped: %.2f,\
                            nav_times:%.2f, intrusion_nums:%.2f, intrusion_times:%.2f,\
                            trajectory_length: %.2f", self.nav_nums, self.success_nums/self.nav_nums,\
                              self.collision_nums/self.success_nums, self.collision_ped_nums, 
                              self.nav_times/self.success_nums,\
                              self.intrusion_nums/self.success_nums, self.intrusion_times/self.success_nums,\
                              self.trajectory_length/self.success_nums)
          self.nav_flag = True
          self.time_start = rospy.Time.now()
          rospy.logwarn("time now: %.2f", self.time_start.to_sec())

      self.time_end = rospy.Time.now()
      if (self.time_end - self.time_start).to_sec() >= self.NAV_TIME:
          rospy.logwarn("nav_failed")
          self.nav_flag = False
          self.trajectory_length_current = 0
          self.collision_current = 0
          self.collision_rate = 0
          self.intrusion_nums_current = 0
          self.intrusion_times_current = 0
          self.collision_ped_current = 0
          self.intrusion_in_flag = False
          self.intrusion_out_flag = False
          #reset robot
          # self._pub_initial_model_state(1, 1, 0)
          # self._pub_initial_model_state(10, 10, 0)
          # time.sleep(3)
          # '''
          # # reset robot odometry:
          # timer = time.time()
          # while time.time() - timer < 0.5:
          #     self._reset_odom_pub.publish(Empty())
          # '''
          # # reset robot inital pose in rviz:
          # self._pub_initial_position(1.5, 0, 0)
          # # self._pub_initial_position(10, 10, 0)
          # time.sleep(3)
      dist_to_goal = np.linalg.norm(
          np.array([
          self.curr_pose.position.x - self.goal_position.x,
          self.curr_pose.position.y - self.goal_position.y,
          self.curr_pose.position.z - self.goal_position.z
          ])
      )
      # settlement
      if dist_to_goal < self.ROBOT_RADIUS + 0.7:
          if self.settle_num == 0:
            rospy.logwarn("nav_success!")
            self.success_nums += 1
            self.nav_flag = False
            self.nav_times += (self.time_end - self.time_start).to_sec()
            self.trajectory_length += self.trajectory_length_current
            self.trajectory_length_current = 0
            self.collision_nums += self.collision_current
            self.collision_current = 0
            self.collision_ped_nums += self.collision_ped_current
            self.collision_ped_current = 0
            self.intrusion_nums += self.intrusion_nums_current
            self.intrusion_nums_current = 0
            self.intrusion_times += self.intrusion_times_current
            self.intrusion_times_current = 0
            self.intrusion_in_flag = False
            self.intrusion_out_flag = False
          self.settle_num += 1
          if self.settle_num == 100:
              self.settle_num = 0 
          

  def _group_callback(self, group_msg):
      self.groups = group_msg

  def _ped_callback(self, ped_msg):
      self.peds = ped_msg

  def _timer2_callback(self, event):
      id = 0
      for group in self.groups.groups:
          id += 1
          # get the centroid
          center = Point()
          center.x = group.centerOfGravity.pose.position.x
          center.y = group.centerOfGravity.pose.position.y
          # calculate the radius of group
          max_radius = -np.Infinity
          for index in group.track_ids:
              ped_i = self.peds.tracks[index - 1]
              h_position = Point()
              h_position.x = ped_i.pose.pose.position.x
              h_position.y = ped_i.pose.pose.position.y
              radius_i = math.sqrt(pow(h_position.x-center.x, 2) +\
                                    pow(h_position.y-center.y, 2))
              if radius_i > max_radius:
                  max_radius = radius_i
          if max_radius > 1.5:
              max_radius = 1.5
          dist_to_intrude = np.linalg.norm(
              np.array([
              0 - center.x,
              0 - center.y,
              ])
              )

          if dist_to_intrude < max_radius and self.intrusion_in_flag == False:
              self.intrusion_in_flag = True
              self.intrusion_group_id = id
              self.intrusion_timer_start = rospy.Time.now()
              rospy.logwarn("an intrusion happenning") 
          if  id==self.intrusion_group_id and self.intrusion_in_flag == True and dist_to_intrude > max_radius:
              self.intrusion_out_flag = True
              if self.intrusion_in_flag == True and self.intrusion_out_flag == True:
                  self.intrusion_nums_current += 1
                  self.intrusion_timer_end = rospy.Time.now()
                  self.intrusion_times_current += (self.intrusion_timer_end - self.intrusion_timer_start).to_sec()/2
                  self.intrusion_in_flag = False
                  self.intrusion_out_flag = False  
                  rospy.logwarn("an intrusion just happened") 
                  rospy.logwarn("intrusion_time: %.2f", self.intrusion_times_current)   
      # for ped_i in self.peds.tracks:
      #   #rospy.logwarn("%d", ped_i.track_id)
      #   #get the position and pose of human_i
      #   h_position = Point()
      #   h_position.x = ped_i.pose.pose.position.x 
      #   h_position.y = ped_i.pose.pose.position.y   
      #   dist_to_ped = np.linalg.norm(
      #   np.array([
      #   0 - h_position.x,
      #   0 - h_position.y,
      #   ])
      #   )  
      #   if dist_to_ped <= self.ROBOT_RADIUS+0.2:
      #       self.collision_ped_current += 1
      #       rospy.logwarn("collision with ped")
            

  def _scan_callback(self, scan_msg):
      scan = scan_msg
      scan = np.array(scan.ranges)
      min_scan_dist = np.amin(scan[scan!=0])
      if min_scan_dist <= self.ROBOT_RADIUS and self.collision_rate==0:
          self.collision_current += 1
          self.collision_rate += 1
          rospy.logwarn("collision!")
      if self.collision_rate != 0:
          self.collision_rate += 1
          if self.collision_rate == 50:
              self.collision_rate = 0

  # for trajectory_length
  def _odom_callback(self, odom_msg):
      self.current_position = odom_msg.pose.pose.position
      if self.previous_position.x != 0.0 and self.previous_position.y != 0.0:
          distance = math.sqrt((self.current_position.x - self.previous_position.x)**2 + (self.current_position.y - self.previous_position.y)**2)
          self.trajectory_length_current += distance
      self.previous_position = self.current_position

  def _map_callback(self, map_msg):
        """
        Receiving map from map topic
        :param: map data
        :return:
        """
        self.map = map_msg
        rospy.logwarn("!get map!")

  def _final_goal_callback(self, final_goal_msg):
      """
      Receiving final goal from final_goal topic
      :param: final goal
      :return:
      """
      self.goal_position = final_goal_msg.pose.position
      rospy.logwarn("get the goal position")

  def _robot_pose_callback(self,  robot_pose_msg):
      self.curr_pose = robot_pose_msg.pose

  def _publish_random_goal(self):
      """
      Publishing new random goal [x, y, theta] for global planner
      :return: goal position [x, y, theta]
      """
      dis_diff = 21
      # only select the random goal in range of 20 m:
      # while(dis_diff >= 12 or dis_diff < 7): # or dis_diff < 1):
      while(dis_diff >=8 or dis_diff<6):
          x, y, theta = self._get_random_pos_on_map(self.map)
          # distance difference：
          dis_diff = np.linalg.norm(
              np.array([
              self.curr_pose.position.x - x,
              self.curr_pose.position.y - y,
              ])
              )
      self._publish_goal(x, y, theta)
      return x, y, theta

  def _publish_goal(self, x, y, theta):
      """
      Publishing goal (x, y, theta)
      :param x x-position of the goal
      :param y y-position of the goal
      :param theta theta-position of the goal
      """
      # publish inital goal:
      goal_yaw = theta
      # create message:
      goal = PoseStamped()
      # initialize header:
      goal.header.stamp = rospy.Time.now()
      goal.header.frame_id = "map"
      # initialize pose:
      # position:
      goal.pose.position.x = x
      goal.pose.position.y = y
      goal.pose.position.z = 0
      # orientation:
      goal.pose.orientation.x = 0
      goal.pose.orientation.y = 0
      goal.pose.orientation.z = np.sin(goal_yaw/2)
      goal.pose.orientation.w = np.cos(goal_yaw/2)
      self._goal_pub.publish(goal)
  
  def _get_random_pos_on_map(self, map):
    """
    Find a valid (free) random position (x, y, theta) on the map
    :param map
    :return: x, y, theta
    """
    map_width = map.info.width * map.info.resolution + map.info.origin.position.x
    map_height = map.info.height * map.info.resolution + map.info.origin.position.y
    x = random.uniform(0.0 , map_width)
    y = random.uniform(0.0, map_height)
    radius = self.ROBOT_RADIUS + 0.5  # safe radius
    while not self._is_pos_valid(x, y, radius, map):
        x = random.uniform(0.0, map_width)
        y = random.uniform(0.0, map_height)

    theta = random.uniform(-math.pi, math.pi)
    return x, y, theta
  
  def _is_pos_valid(self, x, y, radius, map):
      """
      Checks if position (x,y) is a valid position on the map.
      :param  x: x-position
      :param  y: y-position
      :param  radius: the safe radius of the robot 
      :param  map
      :return: True if position is valid
      """
      cell_radius = int(radius/map.info.resolution)
      y_index =  int((y-map.info.origin.position.y)/map.info.resolution)
      x_index =  int((x-map.info.origin.position.x)/map.info.resolution)

      for i in range(x_index-cell_radius, x_index+cell_radius, 1):
          for j in range(y_index-cell_radius, y_index+cell_radius, 1):
              index = j * map.info.width + i
              if index >= len(map.data):
                  return False
              try:
                  val = map.data[index]
              except IndexError:
                  print("IndexError: index: %d, map_length: %d"%(index, len(map.data)))
                  return False
              if val != 0:
                  return False
      return True

  def _pub_initial_model_state(self, x, y, theta):
      """
      Publishing new initial position (x, y, theta) 
      :param x x-position of the robot
      :param y y-position of the robot
      :param theta theta-position of the robot
      """
      robot_state = ModelState()
      robot_state.model_name = "mobile_base"
      robot_state.pose.position.x = x
      robot_state.pose.position.y = y
      robot_state.pose.position.z = 0
      robot_state.pose.orientation.x = 0
      robot_state.pose.orientation.y = 0
      robot_state.pose.orientation.z = np.sin(theta/2)
      robot_state.pose.orientation.w = np.cos(theta/2)
      robot_state.reference_frame = "world"
      # publish model_state to set robot
      # self._set_robot_state_publisher.publish(robot_state)
      rospy.wait_for_service('/gazebo/set_model_state')
      try:
          result = self._set_robot_state_service(robot_state)
          rospy.logwarn("set the model state successfully")
      except rospy.ServiceException:
          rospy.logwarn("/gazebo/set_model_state service call failed")

  def _pub_initial_position(self, x, y, theta):
      """
      Publishing new initial position (x, y, theta) --> for localization
      :param x x-position of the robot
      :param y y-position of the robot
      :param theta theta-position of the robot
      """
      inital_pose = PoseWithCovarianceStamped()
      inital_pose.header.frame_id = "map"
      inital_pose.header.stamp = rospy.Time.now()
      inital_pose.pose.pose.position.x= x
      inital_pose.pose.pose.position.y= y
      inital_pose.pose.pose.position.z= 0
      inital_pose.pose.pose.orientation.x = 0
      inital_pose.pose.pose.orientation.y = 0
      inital_pose.pose.pose.orientation.z = np.sin(theta/2)
      inital_pose.pose.pose.orientation.w = np.cos(theta/2)
      self._initial_pose_pub.publish(inital_pose)


if __name__ == '__main__':
    try:
      rospy.init_node('nav_test')
      nav_test()
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
    except rospy.ROSInterruptException:
      pass
