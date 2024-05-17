import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class WayPointPub:
  def __init__(self):
    self.robot_pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, self.pose_callback)
    self.path_pub = rospy.Publisher('/constant_path', Path, queue_size=1, latch=False)
    self.start_point = PoseStamped()
    self.end_point = PoseStamped()
    self.num_waypoints = 10
    self.start_point.pose.position.x = -6.09
    self.start_point.pose.position.y = 10.02
    self.start_point.pose.position.z = 0
    self.start_point.pose.orientation.x = 0
    self.start_point.pose.orientation.y = 0
    self.start_point.pose.orientation.z = 0
    self.start_point.pose.orientation.w = 1
    self.end_point.pose.position.x = -4.61
    self.end_point.pose.position.y = -7.74
    self.end_point.pose.position.z = 0
    self.start_point.pose.orientation.x = 0
    self.start_point.pose.orientation.y = 0
    self.start_point.pose.orientation.z = 0
    self.start_point.pose.orientation.w = 1
    self.path = Path()
    self.path.header.frame_id = "map"
    self.path.header.stamp = rospy.Time.now()
    step_x = abs(self.start_point.pose.position.x - self.end_point.pose.position.x)/self.num_waypoints
    step_y = abs(self.start_point.pose.position.y - self.end_point.pose.position.y)/self.num_waypoints
    for i in range(self.num_waypoints):
      waypoint = PoseStamped()
      waypoint.pose.position.x = self.start_point.pose.position.x + (i + 1) * step_x
      waypoint.pose.position.y = self.start_point.pose.position.y - (i + 1) * step_y
      waypoint.pose.position.z = 0
      waypoint.pose.orientation.x = 0
      waypoint.pose.orientation.y = 0
      waypoint.pose.orientation.z = 0
      waypoint.pose.orientation.w = 1
      self.path.poses.append(waypoint)
    
  def pose_callback(self, pose_msg):
    robot_pose = PoseStamped()
    robot_pose = pose_msg
    waypoint_top = self.path.poses[0]
    if robot_pose.pose.position.x > waypoint_top.position.x:
      self.path.poses.pop(0)
    self.path_pub.publish(self.path)

if __name__ == '__main__':
  try:
      rospy.init_node('constant_path')
      WayPointPub()
      # spin() simply keeps python from exiting until this node is stopped
      rospy.spin()
  except rospy.ROSInterruptException:
      pass
    
    
    




      
