#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import numpy as np

mavros.set_namespace()

#export PX4_HOME_LAT=40.091754
#export PX4_HOME_LON=-3.695714

class px4Agent:
     def __init__(self):
          self.local_pose_pub = rospy.Publisher(
               "mavros/setpoint_position/local", PoseStamped, queue_size=10)
          self.state_sub = rospy.Subscriber("mavros/state", State, self.state_callback)
          self.state = State()
          self.point = PoseStamped()

          self.timer1 = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
          
          self.local_pos_sub = rospy.Subscriber("mavros/local_position/odom"
               ,Odometry, self.odom_callback)
          
          self.wp = [[2,2,3],[5,5,3],[0,0,3]]
          self.local_position = np.array([0.0, 0.0, 0.0])
          self.wp_mode = False
          self.target_position = np.array([0.0, 0.0, 0.0])

     def odom_callback(self, odom_in):
          self.local_position[0] = odom_in.pose.pose.position.x
          self.local_position[1] = odom_in.pose.pose.position.y
          self.local_position[2] = odom_in.pose.pose.position.z
          
          if self.wp_mode:
               if np.linalg.norm(self.target_position[0:2]-self.local_position[0:2])<1.0:
                    if len(self.wp) > 0:
                         self.waypoint_transition()
                    else:
                         self.setLand()

     def waypoint_transition(self):
          self.target_position = self.wp.pop(0)
          print("Target position is : ", self.target_position)
          self.point.pose.position.x = self.target_position[0]
          self.point.pose.position.y = self.target_position[1]
          self.point.pose.position.z = self.target_position[2]

     def timer_callback(self, event):
          self.local_pose_pub.publish(self.point)
     
     def state_callback(self, state_in):
          self.state = state_in
     
     def setArm(self):
          rospy.wait_for_service("mavros/cmd/arming")
          arm_srv = rospy.ServiceProxy("mavros/cmd/arming",
               CommandBool)
          arm_srv(True)
     
     def setDisarm(self):
          rospy.wait_for_service("mavros/cmd/arming")
          disarm_srv = rospy.ServiceProxy("mavros/cmd/arming",
               CommandBool)
          disarm_srv(False)
     
     def setLand(self):
          rospy.wait_for_service("mavros/cmd/land")
          land_srv = rospy.ServiceProxy("mavros/cmd/land",
               CommandTOL)
          land_srv(yaw=0.0, altitude = 0.0)

     def setMode(self, mode):
          rospy.wait_for_service("mavros/set_mode")
          mode_srv = rospy.ServiceProxy("mavros/set_mode",
               SetMode)
          mode_srv(custom_mode = str(mode))
     
     def startOffboard(self):
          
          self.point.header.frame_id = "map"

          self.point.pose.position.x = 0.0
          self.point.pose.position.y = 0.0
          self.point.pose.position.z = 3.0

          self.point.pose.orientation.x = 0.0
          self.point.pose.orientation.y = 0.0
          self.point.pose.orientation.z = 0.0
          self.point.pose.orientation.w = 1.0

          self.local_pose_pub.publish(self.point)
          self.setMode("OFFBOARD")
          self.setArm() 

          while not rospy.is_shutdown() and not self.state.connected:
               print("wait for fcu")
               time.sleep(0.1)

          while not self.state.armed:
               print("Trying to arm")
               self.setArm()
               time.sleep(0.1)
          
          for i in range(50):
               self.point.header.stamp = rospy.Time.now()
               self.local_pose_pub.publish(self.point)
               self.setMode("OFFBOARD")
               time.sleep(0.1)
          
          self.wp_mode = True
          print("End of offboard")
          
             

class Mission:
     def __init__(self):
          self.drone = px4Agent()
     
     def act(self):
          self.drone.startOffboard()

if __name__ == "__main__":
     rospy.init_node("px4_operation_node", anonymous = False)
     ms = Mission()
     ms.act()
     rospy.spin()







