#!/usr/bin/env python
#
# revision history: xzt
#  20210604 (TE): first version
#
# usage:
#
# This script is to setup the Gym-Gazebo training environment for the turtlebot.
#------------------------------------------------------------------------------

import numpy as np
import numpy.matlib
import random
import math
from scipy.optimize import linprog, minimize
import tf.transformations as tf
#import sympy as sp
import threading

#import itertools 
import rospy
import gym # https://github.com/openai/gym/blob/master/gym/core.py
from gym.utils import seeding
from gym import spaces
#from .gym_gazebo_env import GymGazeboEnv
from .gazebo_connection import GazeboConnection
from std_msgs.msg import Float64, Empty, Bool
from sensor_msgs.msg import LaserScan, PointCloud, PointField
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, Twist, Point, PoseStamped, PoseWithCovarianceStamped
import time
from kobuki_msgs.msg import BumperEvent
from actionlib_msgs.msg import GoalStatusArray
from pedsim_msgs.msg  import TrackedPersons, TrackedPerson, DangerZone, DangerZones, TrackedGroup, TrackedGroups, GMM, GMMGroup
from cnn_msgs.msg import CNN_data
from visualization_msgs.msg import Marker, MarkerArray


class DRLNavEnv(gym.Env):
    """
    Gazebo env converts standard openai gym methods into Gazebo commands

    To check any topic we need to have the simulations running, we need to do two things:
        1)Unpause the simulation: without that the stream of data doesnt flow. This is for simulations
        that are pause for whatever the reason
        2)If the simulation was running already for some reason, we need to reset the controlers.
        This has to do with the fact that some plugins with tf, dont understand the reset of the simulation and need to be reseted to work properly.
    """
    def __init__(self):
        # To reset Simulations
        rospy.logdebug("START init DRLNavEnv")
        self.seed()

        # robot parameters:
        self.ROBOT_RADIUS = 0.4
        self.GOAL_RADIUS = 0.5 #0.3
        self.DIST_NUM = 10
        self.pos_valid_flag = True
        # bumper:
        self.bump_flag = False
        self.bump_num = 0
        # reward:
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        self.num_iterations = 0

        # action limits
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.4
        # observation limits
        # action space
        # self.high_action = np.array([self.max_linear_speed, self.max_angular_speed])
        # self.low_action = np.array([0, -self.max_angular_speed])
        # MaxAbsScaler: normalize to (-1,1)
        self.high_action = np.array([1, 1])
        self.low_action = np.array([-1, -1])
        self.action_space = spaces.Box(low=self.low_action, high=self.high_action, dtype=np.float32)
        # observation space
        self.cnn_data = CNN_data()
        self.lidar_data = LaserScan()
        self.ped_pos = []
        self.scan = []
        self.goal = []
        self.sub_goal = Point()
        self.drl_cmd_vel = Twist()
        self.drl_cmd_last_vel = Twist()
        self.cmd_list = []
        #self.vel = []
       
        # self.observation_space = spaces.Box(low=-30, high=30, shape=(6402,), dtype=np.float32)
        # MaxAbsScaler: normalize to (-1,1)
        self.observation_space = spaces.Box(low=-1, high=1, shape=(19202,), dtype=np.float32)
        # info, initial position and goal position
        self.init_pose = Pose()
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.vel_list = []
        self.last_vel = Twist()
        self.goal_position = Point()
        self.info = {}
        # episode done flag:
        self._episode_done = False
        # goal reached flag:
        self._goal_reached = False
        # reset flag:
        self._reset = True

        # vo algorithm:
        self.mht_peds = TrackedPersons()

        #Gaussian process:
        self.groups = TrackedGroups()
        

        # To reset Simulations
        self.gazebo = GazeboConnection(
        start_init_physics_parameters=True,
        reset_world_or_sim= "WORLD" #"SIMULATION" #
        )

        self.gazebo.unpauseSim()
        # We Start all the ROS related Subscribers and publishers
        self._map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
        self._cnn_data_sub = rospy.Subscriber("/cnn_data", CNN_data, self._cnn_data_callback, queue_size=1, buff_size=2**24)
        self._scan_sub = rospy.Subscriber("/laser", LaserScan, self._scan_callback)
        self._robot_pos_sub = rospy.Subscriber("/robot_pose", PoseStamped, self._robot_pose_callback) #, queue_size=1)
        self._robot_vel_sub = rospy.Subscriber('/odom', Odometry, self._robot_vel_callback) #, queue_size=1)
        self._final_goal_sub = rospy.Subscriber("/move_base/current_goal", PoseStamped, self._final_goal_callback) #, queue_size=1)
        self._goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self._goal_status_callback) #, queue_size=1)
        self._drl_cmd_vel_sub = rospy.Subscriber("teleop_velocity_smoother/raw_cmd_vel", Twist, self._drlcmdvel_callback)
        # self._model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1)
        # self._bumper_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self._bumper_callback) #, queue_size=1)
        # self._dwa_vel_sub = rospy.Subscriber('/move_base/cmd_vel', Twist, self._dwa_vel_callback)
        self._ped_sub = rospy.Subscriber("/track_ped", TrackedPersons, self._ped_callback)
        # Publish velocities:
        # self._cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1, latch=False)
        self._cmd_vel_pub = rospy.Publisher('/drl_cmd_vel', Twist, queue_size=5, latch=False)
        # Publish goal:
        self._initial_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        self._sub_goal_sub = rospy.Subscriber('/cnn_goal', Point, self._sub_goal_callback)
        # Set model state:
        # self._set_robot_state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1, latch=False)
        self._set_robot_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # Set odometry:
        # self._reset_odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1, latch=True)
        # Set initial pose:
        self._initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        # self._done_pub = rospy.Publisher('/drl_done', Bool, queue_size=1, latch=False)
        self._danger_zone_pub = rospy.Publisher('/danger_zone', MarkerArray, queue_size=1)
        self._debugging_pub = rospy.Publisher('/debugging', Float64, queue_size=1)
        # self._norm_visual_pub = rospy.Publisher('/ellipses_marker', MarkerArray, queue_size=10)
        self._groups_sub = rospy.Subscriber('/track_group', TrackedGroups, self._group_callback)
        self._GMM_pub = rospy.Publisher('/GMM', MarkerArray, queue_size=1)
        # self.controllers_object.reset_controllers()
        self._check_all_systems_ready()
        self.gazebo.pauseSim()   
            
        rospy.logdebug("Finished TurtleBot2Env INIT...")
        

    # Env methods
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        """
        Gives env an action to enter the next state,
        obs, reward, done, info = env.step(action)
        """
        # Convert the action num to movement action
        self.gazebo.unpauseSim()
        self._take_action(action)
        self.gazebo.pauseSim()
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._is_done(reward)
        # self._done_pub.publish(done)
        info = self._post_information()
        #print('info=', info, 'reward=', reward, 'done=', done)
        return obs, reward, done, info

    def reset(self):
        """ 
        obs, info = env.reset() 
        """
        # self.gazebo.pauseSim()
        rospy.logdebug("Reseting RobotGazeboEnvironment")
        self._reset_sim()
        obs = self._get_observation()
        info = self._post_information()
        rospy.logdebug("END Reseting RobotGazeboEnvironment")
        return obs

    def close(self):
        """
        Function executed when closing the environment.
        Use it for closing GUIS and other systems that need closing.
        :return:
        """
        rospy.logwarn("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")

    def _reset_sim(self):
        """Resets a simulation
        """
        rospy.logdebug("START robot gazebo _reset_sim")
        self.gazebo.unpauseSim()
        # self.gazebo.resetSim()
        self._set_init()
        self.gazebo.pauseSim()
        rospy.logdebug("END robot gazebo _reset_sim")
        return True
    
    def _set_init(self):
        """ 
        Set initial condition for simulation
        1. Set turtlebot at a random pose inside playground by publishing /gazebo/set_model_state topic
        2. Set a goal point inside playground for red ball
        Returns: 
        init_position: array([x, y]) 
        goal_position: array([x, y])      
        """
        rospy.logdebug("Start initializing robot...")
        # reset the robot velocity to 0:
        self._cmd_vel_pub.publish(Twist())

        #self._reset = True
        # reset simulation to orignal:
        if(self._reset): 
            self._reset = False
            '''
            self.navigation_launcher.restart()
            self.cnn_data_launcher.restart()
            time.sleep(6)
            '''
            self._check_all_systems_ready()
            
            # self.gazebo.resetSim()
            # test whether robot pose is in the map free space:
            self.pos_valid_flag = False
            map = self.map
            while not self.pos_valid_flag:
                # initialize the robot pose:
                seed_initial_pose = random.randint(0, 18)
                self._set_initial_pose(seed_initial_pose)
                # wait for the initial process is ok:
                time.sleep(4)
                # get the current position:
                x = self.curr_pose.position.x
                y = self.curr_pose.position.y
                radius = self.ROBOT_RADIUS
                self.pos_valid_flag = self._is_pos_valid(x, y, radius, map)
                
        # publish inital goal:
        [goal_x, goal_y, goal_yaw] = self._publish_random_goal()

        #[goal_x, goal_y, goal_yaw] = [3, 3, 0]
        #self._publish_goal(3, 3, 0)

        time.sleep(1)    
        self._check_all_systems_ready()

        # initalize info:
        self.init_pose = self.curr_pose # inital_pose.pose.pose
        self.curr_pose = self.curr_pose # inital_pose.pose.pose
        self.goal_position.x = goal_x
        self.goal_position.y = goal_y
        rospy.logwarn("Robot was initiated as {}".format(self.init_pose))

        # reset pose valid flag:
        self.pos_valid_flag = True
        # reset bumper:
        self.bump_flag = False
        # self.bump_num = 0
        # reset the number of iterations:
        self.num_iterations = 0
        # reset distance to goal register:
        self.dist_to_goal_reg = np.zeros(self.DIST_NUM)
        # reset episode done flag:
        self._episode_done = False

        # Give the system a little time to finish initialization
        rospy.logdebug("Finish initialize robot.")

        # start the timer if this is the first path received
        #if self.check_timer is None:
        #    self.check_start()
        
        return self.init_pose, self.goal_position

    def _set_initial_pose(self, seed_initial_pose):
        # if(seed_initial_pose == 0):
        #     # set turtlebot initial pose in gazebo:
        #     self._pub_initial_model_state(1, 1, 0)
        #     time.sleep(1)
        #     '''
        #     # reset robot odometry:
        #     timer = time.time()
        #     while time.time() - timer < 0.5:
        #         self._reset_odom_pub.publish(Empty())
        #     '''
        #     # reset robot inital pose in rviz:
        #     self._pub_initial_position(1, 0, 0)
        # elif(seed_initial_pose == 1):
        #     self._pub_initial_model_state(19, 5, 0)
        #     time.sleep(1)
        #     '''
        #     # reset robot odometry:
        #     timer = time.time()
        #     while time.time() - timer < 0.5:
        #         self._reset_odom_pub.publish(Empty())
        #     '''
        #     # reset robot inital pose in rviz:
        #     self._pub_initial_position(19, 5, 0)
        # else:
        #     self._pub_initial_model_state(1, 1, 0)
        #     time.sleep(1)
        #     '''
        #     # reset robot odometry:
        #     timer = time.time()
        #     while time.time() - timer < 0.5:
        #         self._reset_odom_pub.publish(Empty())
        #     '''
        #     # reset robot inital pose in rviz:
        #     self._pub_initial_position(1, 1, 0)

        if(seed_initial_pose == 0):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(1, 1, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(1, 0, 0.1377)
        elif(seed_initial_pose == 1):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(14, 7, 1.5705)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())     
            ''' 
            # reset robot inital pose in rviz:
            self._pub_initial_position(12.61, 7.5, 1.70)
        elif(seed_initial_pose == 2):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(1, 16, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())   
            '''     
            # reset robot inital pose in rviz:
            self._pub_initial_position(-1, 14.5, 0.13)
        elif(seed_initial_pose == 3):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(14, 22.5, -1.3113)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
              self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(10.7, 23, -1.16)
        elif(seed_initial_pose == 4):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(4, 4, 1.5705)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(3.6, 3.1, 1.70)
        elif(seed_initial_pose == 5):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(2, 9, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(1, 8, 0.13)
        elif(seed_initial_pose == 6):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(30, 9, 3.14)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(28, 11.4, 3.25)
        elif(seed_initial_pose == 7):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(25, 17, 3.14)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(22.5, 18.8, 3.25)
        elif(seed_initial_pose == 8):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(5, 8, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(3.96, 7.47, 0.137)
        elif(seed_initial_pose == 9):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(10, 12, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(8.29, 12.07, 0.116)
        elif(seed_initial_pose == 10):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(14, 15, 1.576)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(11.75, 15.14, 1.729)
        elif(seed_initial_pose == 11):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(18.5, 15.7, 3.14)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(16.06, 16.7, -2.983)
        elif(seed_initial_pose == 12):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(18.5, 11.3, 3.14)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(16.686, 12.434, -2.983)
        elif(seed_initial_pose == 13):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(14, 11.3, 3.14)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(12.142, 12.1, -2.983)
        elif(seed_initial_pose == 14):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(12.5, 13.2, 0.78)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(10.538, 13.418, 0.9285)
        elif(seed_initial_pose == 15):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(12.07, 16.06, 0)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(9.554, 16.216, 0.143)
        elif(seed_initial_pose == 16):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(21, 14, -1.576)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(18.462, 15.352, -1.4276)
        elif(seed_initial_pose == 17):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(14, 22.5, 1.576)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(10.586, 22.836, 1.7089)
        elif(seed_initial_pose == 18):
            # set turtlebot initial pose in gazebo:
            self._pub_initial_model_state(18, 8.5, -1.576)
            time.sleep(1)
            '''
            # reset robot odometry:
            timer = time.time()
            while time.time() - timer < 0.5:
                self._reset_odom_pub.publish(Empty())
            '''
            # reset robot inital pose in rviz:
            self._pub_initial_position(16.551, 9.630, -1.4326)

    # TurtleBotEnv virtual methods
    # ----------------------------
    def _check_all_systems_ready(self):
        """
        Checks that all the subscribers, publishers and other simulation systems are
        operational.
        """
        self._check_all_subscribers_ready()
        self._check_all_publishers_ready()
        return True

    # check subscribers ready:
    def _check_all_subscribers_ready(self):
        rospy.logdebug("START TO CHECK ALL SUBSCRIBERS READY")
        self._check_subscriber_ready("/map", OccupancyGrid)
        self._check_subscriber_ready("/cnn_data", CNN_data)
        self._check_subscriber_ready("/robot_pose", PoseStamped)
        #self._check_subscriber_ready("/mobile_base/commands/velocity", Twist)
        #self._check_subscriber_ready("/move_base/current_goal", PoseStamped)
        self._check_subscriber_ready("/move_base/status", GoalStatusArray)
        #self._check_subscriber_ready("/gazebo/model_states", ModelStates)
        #self._check_subscriber_ready("/mobile_base/events/bumper", BumperEvent)
        rospy.logdebug("ALL SUBSCRIBERS READY")

    def _check_subscriber_ready(self, name, type, timeout=5.0):
        """
        Waits for a sensor topic to get ready for connection
        """
        var = None
        rospy.logdebug("Waiting for '%s' to be READY...", name)        
        while var is None and not rospy.is_shutdown():
            try:
                var = rospy.wait_for_message(name, type, timeout)
                rospy.logdebug("Current '%s' READY=>", name)
            except:
                rospy.logfatal('Sensor topic "%s" is not available. Waiting...', name)
        return var
    
    # check publishers ready:
    def _check_all_publishers_ready(self):
        rospy.logdebug("START TO CHECK ALL PUBLISHERS READY")
        self._check_publisher_ready(self._cmd_vel_pub.name, self._cmd_vel_pub)
        self._check_publisher_ready(self._initial_goal_pub.name, self._initial_goal_pub)
        #self._check_publisher_ready(self._set_robot_state_publisher.name, self._set_robot_state_publisher)
        self._check_service_ready('/gazebo/set_model_state')
        # self._check_publisher_ready(self._reset_odom_pub.name, self._reset_odom_pub)
        self._check_publisher_ready(self._initial_pose_pub.name, self._initial_pose_pub)
        rospy.logdebug("ALL PUBLISHERS READY")

    def _check_publisher_ready(self, name, obj, timeout=5.0):
        """
        Waits for a publisher to get response
        """
        rospy.logdebug("Waiting for '%s' to get response...", name) 
        start_time = rospy.Time.now()
        while obj.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.logfatal('No subscriber found for publisher %s. Exiting', name)
        rospy.logdebug("'%s' Publisher Connected", name)

    def _check_service_ready(self, name, timeout=5.0):
        """
        Waits for a service to get ready
        """
        rospy.logdebug("Waiting for '%s' to be READY...", name)
        try:
            rospy.wait_for_service(name, timeout)
            rospy.logdebug("Current '%s' READY=>", name)
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logfatal("Service '%s' unavailable.", name)

    # Call back functions read subscribed sensors' data
    # ----------------------------
    # Callback function for the map subscriber
    def _map_callback(self, map_msg):
        """
        Receiving map from map topic
        :param: map data
        :return:
        """
        self.map = map_msg

    # Callback function for the cnn_data subscriber
    def _cnn_data_callback(self, cnn_data_msg):
        """
        Receiving cnn data from cnn_data topic
        :param: cnn data
        :return:
        """
        self.cnn_data = cnn_data_msg

    def _scan_callback(self, scan_msg):
        self.lidar_data = scan_msg


    # Callback function for the robot pose subscriber
    def _robot_pose_callback(self, robot_pose_msg):
        """
        Receiving robot pose from robot_pose topic
        :param: robot pose
        :return:
        """
        self.curr_pose = robot_pose_msg.pose
    
    # Callback function for the robot velocity subscriber
    def _robot_vel_callback(self, robot_vel_msg):
        """
        Receiving robot velocity from robot_vel topic
        :param: robot velocity
        :return:
        """
        self.curr_vel = robot_vel_msg.twist.twist
        if len(self.vel_list) > 2:
            self.last_vel = self.vel_list[-2]
        self.vel_list.append(self.curr_vel)
        if len(self.vel_list) > 5:
            self.vel_list.pop(0)
        # rospy.logwarn("Current Velocity: \ncurr_vel = {}".format(self.curr_vel.angular.z))
        # rospy.logwarn("Last Velocity: \nlast_vel = {}".format(self.last_vel.angular.z))

    # Callback function for the final goal subscriber
    def _final_goal_callback(self, final_goal_msg):
        """
        Receiving final goal from final_goal topic
        :param: final goal
        :return:
        """
        self.goal_position = final_goal_msg.pose.position

    '''
    # Callback function for the model states subscriber
    def _model_states_callback(self, model_states_msg):
        """
        Receiving model states from model_states topic
        :param: model states
        :return:
        """
        self.model_states = model_states_msg
    '''
   
    '''
    # Callback function for the bumper subscriber
    def _bumper_callback(self, bumper_msg):
        """
        Receiving bumper data from bumper topic
        :param: bumper data 
        :return:
        """
        #bumper_msg.bumper: LEFT (0), CENTER (1), RIGHT (2)
        #bumper_msg.state: RELEASED(0), PRESSED(1)
        if(bumper_msg.state == BumperEvent.PRESSED):
            self.bump_flag = True
        else:
            pass
        rospy.loginfo("Bumper Event:" + str(bumper_msg.bumper))
    '''

    # Callback function for the goal status subscriber
    def _goal_status_callback(self, goal_status_msg):
        """
        Checking goal status callback from global planner.
        """
        if(len(goal_status_msg.status_list) > 0):
            last_element = goal_status_msg.status_list[-1]
            rospy.logwarn(last_element.text)
            if(last_element.status == 3): # succeeded 
                self._goal_reached = True
            else:
                self._goal_reached = False  
        else:
            self._goal_reached = False       
        return

    # Callback function for the pedestrian subscriber
    def _ped_callback(self, trackPed_msg): 
        self.mht_peds = trackPed_msg

    def _group_callback(self, trackGroup_msg):
        self.groups = trackGroup_msg
    
    def _sub_goal_callback(self, subGoal_msg):
        self.sub_goal = subGoal_msg

    def _drlcmdvel_callback(self, drlcmdvel_msg):
        self.drl_cmd_vel = drlcmdvel_msg
        self.cmd_list.append(self.drl_cmd_vel)
        if len(self.cmd_list) > 2:
            self.drl_cmd_last_vel = self.cmd_list[-2]
        if len(self.cmd_list) > 10:
            self.cmd_list.pop(0)
        rospy.logwarn("drlcmdvel_lastz:%.4f, drlcmdvel_z%.4f", self.drl_cmd_last_vel.angular.z, self.drl_cmd_vel.angular.z)
    # ----------------------------

    # Publisher functions to publish data
    # ----------------------------
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

    def _publish_random_goal(self):
        """
        Publishing new random goal [x, y, theta] for global planner
        :return: goal position [x, y, theta]
        """
        dis_diff = 21
        # only select the random goal in range of 20 m:
        while(dis_diff >= 7 or dis_diff < 4.2): # or dis_diff < 1):
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
        self._initial_goal_pub.publish(goal)

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
    # ----------------------------

    # environment setup:
    def _get_observation(self):
        """
        Returns the observation.
        """
        self.ped_pos = self.cnn_data.ped_pos_map
        self.scan = self.cnn_data.scan
        self.goal = self.cnn_data.goal_cart
        self.vel = self.cnn_data.vel
        
        # ped map:
        # MaxAbsScaler:
        v_min = -2
        v_max = 2
        self.ped_pos = np.array(self.ped_pos, dtype=np.float32)
        self.ped_pos = 2 * (self.ped_pos - v_min) / (v_max - v_min) + (-1)

        # scan map:
        # MaxAbsScaler:
        temp = np.array(self.scan, dtype=np.float32)
        scan_avg = np.zeros((20,80))
        for n in range(10):
            scan_tmp = temp[n*720:(n+1)*720]
            for i in range(80):
                scan_avg[2*n, i] = np.min(scan_tmp[i*9:(i+1)*9])
                scan_avg[2*n+1, i] = np.mean(scan_tmp[i*9:(i+1)*9])
        
        scan_avg = scan_avg.reshape(1600)
        scan_avg_map = np.matlib.repmat(scan_avg,1,4)
        self.scan = scan_avg_map.reshape(6400)
        s_min = 0
        s_max = 30
        self.scan = 2 * (self.scan - s_min) / (s_max - s_min) + (-1)
        
        # goal:
        # MaxAbsScaler:
        g_min = -2
        g_max = 2
        self.goal = np.array(self.goal, dtype=np.float32)
        self.goal = 2 * (self.goal - g_min) / (g_max - g_min) + (-1)
        #self.goal = self.goal.tolist()

        '''
        # vel:
        # MaxAbsScaler:
        vx_min = 0
        vx_max = 0.5
        wz_min = -2
        wz_max = 2
        self.vel = np.array(self.vel, dtype=np.float32)
        self.vel[0] = 2 * (self.vel[0] - vx_min) / (vx_max - vx_min) + (-1)
        self.vel[1] = 2 * (self.vel[1] - wz_min) / (wz_max - wz_min) + (-1)
        '''

        # observation:
        self.observation = np.concatenate((self.ped_pos, self.scan, self.goal), axis=None) #list(itertools.chain(self.ped_pos, self.scan, self.goal))
        #self.observation = np.concatenate((self.scan, self.goal), axis=None)
        rospy.logdebug("Observation ==> {}".format(self.observation))
        return self.observation
    
    def _post_information(self):
        """
        Return:
        info: {"init_pose", "goal_position", "current_pose"}
        """
        self.info = {
            "initial_pose": self.init_pose,
            "goal_position": self.goal_position,
            "current_pose": self.curr_pose
            }
        
        return self.info

    def _take_action(self, action):
        """
        Set linear and angular speed for Turtlebot and execute.
        Args:
        action: 2-d numpy array.
        """
        rospy.logdebug("TurtleBot2 Base Twist Cmd>>\nlinear: {}\nangular: {}".format(action[0], action[1]))
        cmd_vel = Twist()
        # distance to goal:
        #distance = np.linalg.norm(self.goal)
        #if(distance > 0.7):
        # MaxAbsScaler:
        vx_min = 0
        vx_max = 0.5
        vz_min = 0.4 #-2 #-3
        vz_max = 0.4 #2 #3
        cmd_vel.linear.x = (action[0] + 1) * (vx_max - vx_min) / 2 + vx_min
        cmd_vel.angular.z = (action[1] + 1) * (vz_max - vz_min) / 2 + vz_min
        #self._check_publishers_connection()
        rate = rospy.Rate(20)
        for _ in range(1):
            self._cmd_vel_pub.publish(cmd_vel)
            #rospy.logdebug("cmd_vel: \nlinear: {}\nangular: {}".format(cmd_vel.linear.x,
            #                                                    cmd_vel.angular.z))
            rate.sleep()
    
        # self._cmd_vel_pub.publish(cmd_vel)
        # rospy.logwarn("cmd_vel: \nlinear: {}\nangular: {}".format(cmd_vel.linear.x, cmd_vel.angular.z))

    # Compute Reward Section:
    # -------------------------------
    def _compute_reward(self):
        """Calculates the reward to give based on the observations given.
        """
        # reward parameters:
        r_arrival = 4#20 #15
        r_waypoint = 0.25 #3.2 #2.5 #1.6 #2 #3 #1.6 #6 #2.5 #2.5
        r_collision = -20 #-15
        r_scan = -0.2 #-0.15 #-0.3
        r_angle = 0.6 #0.5 #1 #0.8 #1 #0.5
        r_rotation = -0.05 #-0.15 #-0.4 #-0.5 #-0.2 # 0.1
        r_shake = -0.05

        angle_thresh = np.pi/6
        w_thresh = 0.3 # 0.7
        shake_thresh = 0.15

        # reward parts:
        r_g = self._goal_reached_reward(r_arrival, r_waypoint)
        #r_g = self._goal_reached_dz_reward()
        # r_c = self._obstacle_collision_dz_punish(self.lidar_data, -0.05, -1.0)
        #r_c = self._obstacle_collision_punish(self.cnn_data.scan[-720:], r_scan, r_collision)
        r_w = self._angular_velocity_punish(self.drl_cmd_vel.angular.z, r_rotation, w_thresh)
        r_s = self._angular_shake_punish(self.drl_cmd_vel.angular.z, self.drl_cmd_last_vel.angular.z, r_shake, shake_thresh)
        #r_t = self._theta_reward(self.goal, self.mht_peds, self.curr_vel.linear.x, r_angle, angle_thresh)
        #r_d = self._danger_zone_punish(self.mht_peds, 0.75, 0.35)
        r_n = self._GMM_punish(self.groups, self.mht_peds, -1.0)
        reward = r_g + r_n + r_w + r_s #+ r_c #+ r_d #r_w + r_d  #+ r_t#+ r_v # + r_p
        rospy.logwarn("Current Velocity: curr_vel = {}".format(self.drl_cmd_vel.angular.z))
        rospy.logwarn("Last Velocity: last_vel = {}".format(self.drl_cmd_last_vel.angular.z))
        #rospy.logwarn("Compute reward done. \nreward = {}".format(reward))
        return reward

    def _goal_reached_reward(self, r_arrival, r_waypoint):
        """
        Returns positive reward if the robot reaches the goal.
        :param transformed_goal goal position in robot frame
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        # distance to goal:
        dist_to_goal = np.linalg.norm(
            np.array([
            self.curr_pose.position.x - self.goal_position.x,
            self.curr_pose.position.y - self.goal_position.y,
            self.curr_pose.position.z - self.goal_position.z
            ])
        )     
        # t-1 id:
        t_1 = self.num_iterations % self.DIST_NUM
        # initialize the dist_to_goal_reg:
        if(self.num_iterations == 0):
            self.dist_to_goal_reg = np.ones(self.DIST_NUM)*dist_to_goal

        # rospy.logwarn("distance_to_goal_reg = {}".format(self.dist_to_goal_reg[t_1]))
        # rospy.logwarn("distance_to_goal = {}".format(dist_to_goal))
        max_iteration = 512 #800 
        # reward calculation:
        if(dist_to_goal <= self.GOAL_RADIUS):  # goal reached: t = T
            reward = r_arrival
        elif(self.num_iterations >= max_iteration):  # failed to the goal
            reward = -r_arrival     
        else:   # on the way
            reward = r_waypoint*(self.dist_to_goal_reg[t_1] - dist_to_goal)

        # storage the robot pose at t-1:
        #if(self.num_iterations % 40 == 0):
        self.dist_to_goal_reg[t_1] = dist_to_goal #self.curr_pose
    
        rospy.logwarn("Goal reached reward: {}".format(reward))
        return reward

    def _goal_reached_dz_reward(self):
        dist_to_goal = np.linalg.norm(
            np.array([
            self.curr_pose.position.x - self.goal_position.x,
            self.curr_pose.position.y - self.goal_position.y,
            self.curr_pose.position.z - self.goal_position.z
            ])
        )
        if dist_to_goal < self.GOAL_RADIUS:
            reward = 1
        else:
            reward = 0
        return reward         
        
    def _obstacle_collision_punish(self, scan, r_scan, r_collision):
        """
        Returns negative reward if the robot collides with obstacles.
        :param scan containing obstacles that should be considered
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.02):
            reward = r_collision
        elif(min_scan_dist < 3*self.ROBOT_RADIUS):
            reward = r_scan * (3*self.ROBOT_RADIUS - min_scan_dist)
        else:
            reward = 0.0
        self._debugging_pub.publish(min_scan_dist)
        rospy.logwarn("Obstacle collision reward: {}".format(reward))
        return reward
    
    def _obstacle_collision_dz_punish(self, scan, r_scan, r_collision):
        """
        Returns negative reward if the robot collides with obstacles.
        :param scan containing obstacles that should be considered
        :param k reward constant
        :return: returns reward colliding with obstacles
        """
        scan = np.array(scan.ranges)
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.02):
            reward = r_collision
        elif(min_scan_dist < 3*self.ROBOT_RADIUS):
            reward = r_scan * (3*self.ROBOT_RADIUS - min_scan_dist)
        else:
            reward = 0.0
        self._debugging_pub.publish(min_scan_dist)
        rospy.logwarn("Obstacle collision reward: {}".format(reward))
        return reward
    

    def _angular_velocity_punish(self, w_z, r_rotation, w_thresh):
        """
        Returns negative reward if the robot turns.
        :param w roatational speed of the robot
        :param fac weight of reward punish for turning
        :param thresh rotational speed > thresh will be punished
        :return: returns reward for turning
        """
        if(abs(w_z) > w_thresh):
            reward = -abs(w_z * r_rotation)
        else:
            reward = 0.0

        rospy.logwarn("rotation punish: %.5f", reward)
        return reward

    def _angular_shake_punish(self, current_wz, last_wz, r_shake, shake_thresh):
        if(abs(current_wz - last_wz) > shake_thresh):
            reward = abs(current_wz - last_wz) * r_shake
        else:
            reward = 0.0

        rospy.logwarn("shake punish reward: {}".format(reward))
        return reward

    def _theta_reward(self, goal, mht_peds, v_x, r_angle, angle_thresh):
        """
        Returns negative reward if the robot turns.
        :param w roatational speed of the robot
        :param fac weight of reward punish for turning
        :param thresh rotational speed > thresh will be punished
        :return: returns reward for turning
        """
        # prefer goal theta:
        theta_pre = np.arctan2(goal[1], goal[0])
        d_theta = theta_pre

        # get the pedstrain's position:
        if(len(mht_peds.tracks) != 0):  # tracker results
            d_theta = np.pi/2 #theta_pre
            N = 60
            theta_min = 1000
            for i in range(N):
                theta = random.uniform(-np.pi, np.pi)
                free = True
                for ped in mht_peds.tracks:
                    #ped_id = ped.track_id 
                    # create pedestrian's postion costmap: 10*10 m
                    p_x = ped.pose.pose.position.x
                    p_y = ped.pose.pose.position.y
                    p_vx = ped.twist.twist.linear.x
                    p_vy = ped.twist.twist.linear.y
                    
                    ped_dis = np.linalg.norm([p_x, p_y])
                    if(ped_dis <= 7):
                        ped_theta = np.arctan2(p_y, p_x)
                        vo_theta = np.arctan2(3*self.ROBOT_RADIUS, np.sqrt(ped_dis**2 - (3*self.ROBOT_RADIUS)**2))
                        # collision cone:
                        theta_rp = np.arctan2(v_x*np.sin(theta)-p_vy, v_x*np.cos(theta) - p_vx)
                        if(theta_rp >= (ped_theta - vo_theta) and theta_rp <= (ped_theta + vo_theta)):
                            free = False
                            break

                # reachable available theta:
                if(free):
                    theta_diff = (theta - theta_pre)**2
                    if(theta_diff < theta_min):
                        theta_min = theta_diff
                        d_theta = theta
                
        else: # no obstacles:
            d_theta = theta_pre

        reward = r_angle*(angle_thresh - abs(d_theta))

        rospy.logwarn("Theta reward: {}".format(reward))
        return reward  

    def _danger_zone_punish(self, mht_peds, r_static, m_v):
        """
        Returns negative reward if the robot intrudes into the danger zone
        :param mht_peds: information of tracked persons
        :param r_static: the basic radius of human
        :param m_v: the rate of increase in radius with velocity
        :return: returns reward 
        """
        reward = 0
        human_radius = 0.3 # radius of human
        back_dist = 0.5
        id = 0
        in_DZ_flag = False
        dangerzone_makerarray = MarkerArray()
        for ped in mht_peds.tracks:
          dangerzone = DangerZone()
          vx = ped.twist.twist.linear.x  
          vy = ped.twist.twist.linear.y 
          px = ped.pose.pose.position.x 
          py = ped.pose.pose.position.y 
          v = math.sqrt((vx * vx) + (vy * vy))
          beta = math.atan2(vy, vx) # direction angle
          if beta < 0:
              beta = beta + 2 * math.pi
          r0 = m_v * v + r_static # init radius
          #theta = (11 * math.pi / 6) * math.exp(-1.4 * v) + math.pi / 6
          theta = (11 * math.pi / 6) * math.exp(-1.05 * v) + math.pi / 6

          point_B = Point()
          point_C = Point()
          point_D = Point()
          point_B.x = px + r0 * math.cos(beta + theta / 2)
          point_B.y = py + r0 * math.sin(beta + theta / 2)
          point_C.x = px + r0 * math.cos(beta - theta / 2)
          point_C.y = py + r0 * math.sin(beta - theta / 2)
          point_D.x = (point_B.x+point_C.x)/2
          point_D.y = (point_B.y+point_C.y)/2

          if theta>math.pi and math.sqrt(pow((px-point_D.x),2)+pow(py-point_D.y,2))>human_radius:
            point_L = Point()
            point_L.x = px
            point_L.y = py

            dangerzone.point_L = point_L
            dangerzone.radius = r0
            dangerzone.d_angle = beta
            dangerzone.size_angle = 2*math.pi
            dangerzone.id = id
            id = id + 1
          elif theta > math.pi:
              D = math.sqrt(pow((point_B.x-point_C.x),2)+pow((point_B.y-point_C.y),2))/2
              d = math.sqrt(pow((px-point_D.x),2)+pow((py-point_D.y),2))
            
              point_L = Point()
              point_L.x = ((human_radius + back_dist) * point_D.x -\
                  (human_radius + back_dist - d) * px) / d
              point_L.y = ((human_radius + back_dist) * point_D.y -\
                  (human_radius + back_dist - d) * py) / d
              alpha = 2*math.atan2(D, human_radius+back_dist-d)

              dangerzone.point_L = point_L
              dangerzone.radius = r0 + human_radius + back_dist
              dangerzone.d_angle = beta
              dangerzone.size_angle = alpha
              dangerzone.id = id
              id = id + 1
          else:
              D = math.sqrt(pow((point_B.x-point_C.x),2)+pow((point_B.y-point_C.y),2))/2
              d = math.sqrt(pow((px-point_D.x),2)+pow((py-point_D.y),2))

              point_L = Point()
              point_L.x = ((human_radius+back_dist+d)*px -\
                            (human_radius+back_dist)*point_D.x)/d
              point_L.y = ((human_radius+back_dist+d)*py -\
                            (human_radius+back_dist)*point_D.y)/d
              alpha = 2*math.atan2(D, d+human_radius+back_dist)              

              dangerzone.point_L = point_L
              dangerzone.radius = r0 + human_radius + back_dist
              dangerzone.d_angle = beta
              dangerzone.size_angle = alpha
              dangerzone.id = id
              id = id + 1


          dist_robot_TO_ped = math.sqrt(pow(point_L.x, 2) + pow(point_L.y, 2))
          # for visualize
          marker = Marker()
          marker.header.frame_id = "base_footprint"
          marker.header.stamp = rospy.Time.now()
          marker.ns = "fan_shapes"
          marker.id = dangerzone.id
          marker.type = Marker.LINE_STRIP
          marker.action = Marker.ADD
          marker.pose.orientation.w = 1.0
          marker.scale.x = 0.05
          if dist_robot_TO_ped <= 5:
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
          else:
            marker.color.a = 0.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 0.0

          center = Point(x=dangerzone.point_L.x, y=dangerzone.point_L.y, z=dangerzone.point_L.z)
          radius = dangerzone.radius
          start_angle = dangerzone.d_angle - dangerzone.size_angle / 2
          end_angle = dangerzone.d_angle + dangerzone.size_angle / 2
          num_points = 10
          angle_incr = (end_angle - start_angle) / num_points

          point = Point()
          point.x = center.x
          point.y = center.y
          point.z = center.z
          marker.points.append(point)

          for j in range(num_points + 1):
              point = Point()
              point.x = center.x + radius * math.cos(start_angle + j * angle_incr)
              point.y = center.y + radius * math.sin(start_angle + j * angle_incr)
              point.z = center.z
              marker.points.append(point)

          start_point = Point()
          start_point.x = center.x
          start_point.y = center.y
          start_point.z = center.z
          marker.points.append(start_point)
          dangerzone_makerarray.markers.append(marker)

          vector_ped_TO_robot = Point(-point_L.x, -point_L.y, 0)
          ped_to_robot_angle = math.atan2(vector_ped_TO_robot.y, vector_ped_TO_robot.x)            
          if ped_to_robot_angle < 0:
              ped_to_robot_angle = ped_to_robot_angle + 2 * math.pi
          
          dc = dist_robot_TO_ped - human_radius - self.ROBOT_RADIUS
          if dc < (dangerzone.radius - human_radius) and\
            ped_to_robot_angle > (dangerzone.d_angle - dangerzone.size_angle/2)\
          and ped_to_robot_angle < (dangerzone.d_angle + dangerzone.size_angle/2):
              in_DZ_flag = True
            
          if in_DZ_flag:
              #reward = reward - 50 * (dc * (-0.1) / (r0 - human_radius) + 0.1)
              reward = reward - (dc * (-0.1) / (r0 - human_radius) + 0.1)
          in_DZ_flag = False

        rospy.logwarn("Danger zone punish reward: {}".format(reward))
        self._danger_zone_pub.publish(dangerzone_makerarray)
        return reward

    def _GMM_punish(self, groups, mht_peds, max_punish):
        """
        Returns negative reward if the robot violates the social norms
        :param groups: the tracked human groups
        :param mht_peds: information of tracked persons
        :param max_punish: max punish  
        :return: reward 
        """

        ped_list = np.arange(1, 35, 1).tolist()
        reward = 0
        human_radius = 0.3
        p_space_width_front = 0.25
        p_space_width_side = 0.10
        GMM_markerarray = MarkerArray()
        marker_id = 0
        for group in groups.groups:
            # get the centroid
            center = Point()
            center.x = group.centerOfGravity.pose.position.x
            center.y = group.centerOfGravity.pose.position.y
            # calculate the radius of group
            max_radius = -np.Infinity
            g_velocity = Point()
            humen_nums = 0
            for index in group.track_ids:
                humen_nums += 1
                ped_list.remove(index)
                ped_i = mht_peds.tracks[index - 1]
                h_position = Point()
                h_position.x = ped_i.pose.pose.position.x
                h_position.y = ped_i.pose.pose.position.y
                g_velocity.x += ped_i.twist.twist.linear.x
                g_velocity.y += ped_i.twist.twist.linear.y
                radius_i = math.sqrt(pow(h_position.x-center.x, 2) +\
                                     pow(h_position.y-center.y, 2))
                if radius_i > max_radius:
                    max_radius = radius_i
            # define the GMM_group
            g_GMM = GMMGroup()
            g_velocity.x = g_velocity.x / humen_nums
            g_velocity.y = g_velocity.y / humen_nums
            g_pose = math.atan2(g_velocity.y, g_velocity.x)
            if g_pose < 0:
                g_pose += 2 * math.pi
            relative_pose = math.atan2(-center.x, -center.y)
            if relative_pose < 0:
                relative_pose += 2*math.pi
            relative_pose = relative_pose - g_pose

            g_GMM.position = center
            g_GMM.radius = max_radius
            g_GMM.delta_x = (g_GMM.radius + human_radius + p_space_width_front)/\
              math.sqrt(math.log(2))  #0.64
            g_GMM.delta_y = (g_GMM.radius + human_radius + p_space_width_side)/\
              math.sqrt(math.log(2))  #0.64
            # calculate the collision probability
            d = math.sqrt(pow(0-center.x, 2) + pow(0-center.y, 2)) - self.ROBOT_RADIUS
            if d > g_GMM.radius:
                prob = 0
                if relative_pose > 0.5 * math.pi and relative_pose < 1.5 * math.pi: 
                  prob = math.exp(-(pow(d/(math.sqrt(2)*g_GMM.delta_x), 2) +\
                                    pow(d/(math.sqrt(2)*g_GMM.delta_y), 2)))
                else:
                  prob = math.exp(-(pow(d/(math.sqrt(2)*g_GMM.delta_y), 2) +\
                                    pow(d/(math.sqrt(2)*g_GMM.delta_y), 2)))
                if prob >= 0.5: 
                  reward += prob * max_punish
            elif d <= g_GMM.radius:
                reward += 2 * max_punish

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "circle"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = center.x
            marker.pose.position.y = center.y
            marker.pose.position.z = 0
            quaternion = tf.quaternion_from_euler(0.0, 0.0, g_pose)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.scale.x = 2 * (g_GMM.radius + human_radius + p_space_width_front)
            marker.scale.y = 2 * (g_GMM.radius + human_radius + p_space_width_side)
            marker.scale.z = 0.01
            marker.color.a = 0.5
            marker.color.r = 1.0  
            marker.color.g = 0.0
            marker.color.b = 0.0
            GMM_markerarray.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "circle"
            marker.id = marker_id + 30
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = center.x
            marker.pose.position.y = center.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * g_GMM.radius
            marker.scale.y = 2 * g_GMM.radius
            marker.scale.z = 0.01
            marker.color.a = 0.5
            marker.color.r = 0.0  
            marker.color.g = 0.0
            marker.color.b = 1.0
            GMM_markerarray.markers.append(marker)

            marker_id += 1


        f0 = 1.0
        fv = 11 / 6 #0.5 #0.8  #0.66 #1.33 
        fi = 0.55 #0.7 #0.3 # 0.25
        for index in ped_list:
            ped_i = mht_peds.tracks[index -1]
            #get the position and pose of human_i
            h_position = Point()
            h_velocity = Twist()
            h_speed = 0
            h_pose = 0
            h_position.x = ped_i.pose.pose.position.x 
            h_position.y = ped_i.pose.pose.position.y     
            h_velocity.linear.x = ped_i.twist.twist.linear.x
            h_velocity.linear.y = ped_i.twist.twist.linear.y
            h_speed = math.sqrt(pow(h_velocity.linear.x, 2) + pow(h_velocity.linear.y, 2))
            #rospy.logwarn("human_velocity:%.3f, index: %d", h_speed, index)
            h_pose = math.atan2(h_velocity.linear.y, h_velocity.linear.x)
            if h_pose < 0:
                h_pose += 2 * math.pi
            # get the relative_pose
            relative_pose = math.atan2(0-h_position.y, 0-h_position.x)
            if relative_pose < 0:
                relative_pose += 2*math.pi
            relative_pose = relative_pose - h_pose
            # get the covariance matrix
            delta_x = (f0 + h_speed * fv) * fi * 1.0
            delta_y = fi * 1.0
            # define the GMM of individual
            h_GMM = GMM()
            h_GMM.delta_x = delta_x
            h_GMM.delta_y = delta_y
            h_GMM.position = h_position
            h_GMM.theta = relative_pose
            h_GMM.id = index
            # calculate the collision probability
            d = math.sqrt(pow(0-h_GMM.position.x, 2) + pow(0-h_GMM.position.y, 2)) - self.ROBOT_RADIUS
            if h_GMM.theta > 0.5*math.pi and h_GMM.theta < 1.5*math.pi:
              prob = math.exp(-(pow((d*math.cos(h_GMM.theta))/(math.sqrt(2)*h_GMM.delta_y), 2) +\
                                pow((d*math.sin(h_GMM.theta))/(math.sqrt(2)*h_GMM.delta_y), 2)))
            else:
              prob = math.exp(-(pow((d*math.cos(h_GMM.theta))/(math.sqrt(2)*h_GMM.delta_x), 2) +\
                                pow((d*math.sin(h_GMM.theta))/(math.sqrt(2)*h_GMM.delta_y), 2)))
            if prob >= 0.5:
                reward += prob * max_punish

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "circle"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = h_GMM.position.x
            marker.pose.position.y = h_GMM.position.y
            marker.pose.position.z = 0
            quaternion = tf.quaternion_from_euler(0.0, 0.0, h_pose)
            #rospy.logwarn("h_pose:%.4f", h_pose)
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.scale.x = 2 * math.sqrt(2 * h_GMM.delta_x * h_GMM.delta_y * math.log(2))
            marker.scale.y = 2 * math.sqrt(2 * h_GMM.delta_y * h_GMM.delta_y * math.log(2))
            marker.scale.z = 0.01
            marker.color.a = 0.5
            marker.color.r = 1.0  
            marker.color.g = 0.0
            marker.color.b = 0.0
            GMM_markerarray.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "circle"
            marker.id = marker_id + 30
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = h_GMM.position.x
            marker.pose.position.y = h_GMM.position.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2 * math.sqrt(2 * h_GMM.delta_y * h_GMM.delta_y * math.log(2))
            marker.scale.y = 2 * math.sqrt(2 * h_GMM.delta_y * h_GMM.delta_y * math.log(2))
            marker.scale.z = 0.01
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            GMM_markerarray.markers.append(marker)
            marker_id += 1

        self._GMM_pub.publish(GMM_markerarray)
        rospy.logwarn("GMM punish: %.4f", reward)
        return reward

    def _social_norm_punish(self, groups, mht_peds, max_punish):
        """
        Returns negative reward if the robot violates the social norms
        :param groups: the tracked human groups
        :param mht_peds: information of tracked persons
        :param max_punish: max punish  
        :return: reward 
        """
        ped_list = np.arange(1, 35, 1).tolist()
        reward = 0
        m = 0.4
        d0 = 0.6
        delta_d = 0.5
        marker_array = MarkerArray()
        group_id = 0

        # group nrom
        for group in groups.groups:
            group_id += 1
            # get the centroid
            center = Point()
            center.x = group.centerOfGravity.pose.position.x
            center.y = group.centerOfGravity.pose.position.y
            nums_human = 0
            velocity = Twist()
            for index in group.track_ids:
                ped_list.remove(index)
                ped_i = mht_peds.tracks[index - 1]
                velocity.linear.x += ped_i.twist.twist.linear.x
                velocity.linear.y += ped_i.twist.twist.linear.y
                nums_human += 1
            velocity.linear.x = velocity.linear.x / nums_human
            velocity.linear.y = velocity.linear.y / nums_human
            velocity_angle = math.atan2(velocity.linear.y, velocity.linear.x)
            if velocity_angle < 0:
                velocity_angle += 2*math.pi
            v = math.sqrt(pow(velocity.linear.x, 2) + pow(velocity.linear.y, 2))
            d = d0 + delta_d * (nums_human - 1)
            a = m * v + d
            b = d
            c = math.sqrt(a * a - b * b)
            focus1 = focus2 = Point()
            focus1.x = center.x + c * math.cos(velocity_angle)
            focus1.y = center.y + c * math.sin(velocity_angle)
            focus2.x = center.x - c * math.cos(velocity_angle)
            focus2.y = center.y - c * math.sin(velocity_angle)
            dist = math.sqrt(pow(center.x, 2) + pow(center.y, 2))

            angle_g_r = math.atan2(-center.y, -center.x)
            if angle_g_r < 0:
                angle_g_r += 2*math.pi
            if angle_g_r > velocity_angle - math.pi/2 and \
              angle_g_r < velocity_angle + math.pi/2:
                if (self.dist_point_point(focus1, Point()) +\
                  self.dist_point_point(focus2, Point())) < 2 * a:
                    prob = math.exp(-4 * dist)
                    reward += prob * max_punish
            else:
                if dist < d:
                    prob = math.exp(-4 * dist)
                    reward += prob * max_punish
            # for visualize
            # ellipse_marker = Marker()
            # ellipse_marker.header.frame_id = "base_footprint"
            # ellipse_marker.type = Marker.LINE_STRIP
            # ellipse_marker.action = Marker.ADD
            # ellipse_marker.id = group_id
            # ellipse_marker.scale.x = 0.05
            # ellipse_marker.color.a = 1.0  
            # ellipse_marker.color.r = 1.0  
            # ellipse_marker.color.g = 0.0
            # ellipse_marker.color.b = 0.0

            # num_points = 36
            # for i in range(num_points):
            #     angle_i = 2 * math.pi * i / num_points
            #     x = center.x + a * math.cos(angle_i) * math.cos(velocity_angle) -\
            #       b * math.sin(angle_i) * math.sin(velocity_angle)
            #     y = center.y + a * math.cos(angle_i) * math.sin(velocity_angle) +\
            #       b * math.sin(angle_i) * math.cos(velocity_angle)
            #     p = Point()
            #     p.x = x
            #     p.y = y
            #     ellipse_marker.points.append(p)

            # marker_array.markers.append(ellipse_marker)

        human_id = group_id        
        # individual nrom
        for index in ped_list:
            human_id += 1
            ped_i = mht_peds.tracks[index -1]
            velocity = Twist()
            position = Point()
            velocity.linear.x = ped_i.twist.twist.linear.x      
            velocity.linear.y = ped_i.twist.twist.linear.y
            position.x = ped_i.pose.pose.position.x
            position.y = ped_i.pose.pose.position.y
            velocity_angle = math.atan2(velocity.linear.y, velocity.linear.x)
            v = math.sqrt(pow(velocity.linear.x, 2) + pow(velocity.linear.y, 2))
            d = d0
            a = m * v + d
            b = d
            c = math.sqrt(a * a - b * b)
            focus1 = focus2 = Point()
            focus1.x = position.x + c * math.cos(velocity_angle)
            focus1.y = position.y + c * math.sin(velocity_angle)
            focus2.x = position.x - c * math.cos(velocity_angle)
            focus2.y = position.y - c * math.sin(velocity_angle)
            
            dist = math.sqrt(pow(position.x, 2) + pow(position.y, 2))
            if math.atan2(position.y, position.x) > velocity_angle - 0.5 * math.pi and\
              math.atan2(position.y, position.x) < velocity_angle + 0.5 * math.pi:
              if (self.dist_point_point(focus1, Point()) +\
                self.dist_point_point(focus2, Point())) < 2 * a:
                prob = math.exp(- 4 * dist)
                reward += prob * max_punish
            else:
              if dist < d:
                prob = math.exp(- 4 * dist)
                reward += prob * max_punish      
            
            # ellipse_marker = Marker()
            # ellipse_marker.header.frame_id = "base_footprint"
            # ellipse_marker.type = Marker.LINE_STRIP
            # ellipse_marker.action = Marker.ADD
            # ellipse_marker.id = human_id
            # ellipse_marker.scale.x = 0.05
            # ellipse_marker.color.a = 1.0  
            # ellipse_marker.color.r = 1.0  
            # ellipse_marker.color.g = 0.0
            # ellipse_marker.color.b = 0.0

            # num_points = 36
            # for i in range(num_points):
            #     angle_i = 2 * math.pi * i / num_points
            #     x = position.x + a * math.cos(angle_i) * math.cos(velocity_angle) -\
            #       b * math.sin(angle_i) * math.sin(velocity_angle)
            #     y = position.y + a * math.cos(angle_i) * math.sin(velocity_angle) +\
            #       b * math.sin(angle_i) * math.cos(velocity_angle)
            #     p = Point()
            #     p.x = x
            #     p.y = y
            #     ellipse_marker.points.append(p)

            # marker_array.markers.append(ellipse_marker)

        # self._norm_visual_pub.publish(marker_array)
        rospy.logwarn("social norm punish: %.4f", reward)
        return reward

    def dist_point_point(self, point1, point2):
        dist = 0
        dist = math.sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2))
        return dist

    def _is_done(self, reward):
        """
        Checks if end of episode is reached. It is reached,
        if the goal is reached,
        if the robot collided with obstacle
        if the reward function returns a high negative value.
        if maximum number of iterations is reached,
        :param current state
        :return: True if self._episode_done
        """
        # updata the number of iterations:
        self.num_iterations += 1
        #rospy.logwarn("\nnum_iterations = {}".format(self.num_iterations))

        # 1) Goal reached?
        # distance to goal:
        dist_to_goal = np.linalg.norm(
            np.array([
            self.curr_pose.position.x - self.goal_position.x,
            self.curr_pose.position.y - self.goal_position.y,
            self.curr_pose.position.z - self.goal_position.z
            ])
        )
        if(dist_to_goal <= self.GOAL_RADIUS + 0.2):
            # reset the robot velocity to 0:
            self._cmd_vel_pub.publish(Twist())
            self._episode_done = True
            rospy.logwarn("\n!!!\nTurtlebot went to the goal\n!!!")
            return True

        # 2) Obstacle collision?
        scan = self.cnn_data.scan[-720:]
        min_scan_dist = np.amin(scan[scan!=0])
        #if(self.bump_flag == True): #or self.pos_valid_flag == False):
        if(min_scan_dist <= self.ROBOT_RADIUS and min_scan_dist >= 0.02):
            self.bump_num += 1

        # stop and reset if more than 5 collisions: 
        if(self.bump_num >= 3):
            # reset the robot velocity to 0:
            self._cmd_vel_pub.publish(Twist())
            self.bump_num = 0
            self._episode_done = True
            self._reset = True # reset the simulation world
            rospy.logwarn("TurtleBot collided to obstacles many times before going to the goal @ {}...".format(self.goal_position))
            return True


        # 4) maximum number of iterations?
        max_iteration = 512 #800 # 34 pedestrians: 3000: 5m; no pedestrians: 800: 3m
        if(self.num_iterations > max_iteration):
            # reset the robot velocity to 0:
            self._cmd_vel_pub.publish(Twist())
            self._episode_done = True
            self._reset = True
            rospy.logwarn("TurtleBot got a the maximum number of iterations before going to the goal @ {}...".format(self.goal_position))
            return True

        return False #self._episode_done

    # -------------------------------
    def _env_setup(self, initial_qpos):
        """Initial configuration of the environment. Can be used to configure initial state
        and extract information from the simulation.
        """
        raise NotImplementedError()

