#!/usr/bin/env python3

import rospy
import mavros
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32
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
          self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped,
               self.pose_callback)

          self.local_position = np.array([0.0, 0.0, 0.0])
          self.target_position = np.array([0.0, 0.0, 0.0])
          self.wp = []
          self.wp_mode = False
     
     def get_local_position(self):
          return self.local_position
     
     def update_waypoint(self, wp_in):
          self.wp.append(wp_in)
         

     def pose_callback(self, pose_in):
          self.local_position[0] = pose_in.pose.position.x
          self.local_position[1] = pose_in.pose.position.y
          self.local_position[2] = pose_in.pose.position.z

          if self.wp_mode:
               if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1:
                    if len(self.wp)>0:
                         self.waypoint_transition()

     def waypoint_transition(self):
          target = self.wp.pop(0)
          self.target_position[0] = target[0]
          self.target_position[1] = target[1]
          self.target_position[2] = target[2]
          yaw = target[3]

          print("Goto x: {} y: {} z : {} yaw: {}"
               .format(target[0], target[1], target[2], target[3]))

          self.local_pos_send(self.target_position[0], 
                              self.target_position[1], 
                              self.target_position[2],
                              yaw)

     def get_quaternion_from_euler(self, roll, pitch, yaw):
          qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
          qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
          qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
          qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     
          return [qx, qy, qz, qw]

     
     def local_pos_send(self, x,y,z, yaw = 0.0):
          self.point.pose.position.x = x
          self.point.pose.position.y = y
          self.point.pose.position.z = z

          yaw_rad = yaw / 180.0 *  np.pi

          q = self.get_quaternion_from_euler(0.0, 0.0, yaw_rad)
          self.point.pose.orientation.x = q[0]
          self.point.pose.orientation.y = q[1]
          self.point.pose.orientation.z = q[2]
          self.point.pose.orientation.w = q[3]

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
          self.point.pose.position.z = 4.0

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
          
             

class Mission:
     def __init__(self):
          self.drone = px4Agent()
          self.dist_sub = rospy.Subscriber("color_detection/distance",
               Float32, self.dist_callback)
          self.angle_sub = rospy.Subscriber("color_detection/angle",
               Float32, self.angle_callback)

          self.d = 0.0
          self.theta = 0.0
     
     def act(self):
          self.drone.startOffboard()
          self.drone.local_pos_send(0.0, 0.0, 15.0)
          time.sleep(5)
          target_point = self.calculate_pos(self.d, self.theta)
          self.drone.local_pos_send(target_point[0],target_point[1],3.0)
     def dist_callback(self, dist_in):
          self.d = dist_in.data
          
     def angle_callback(self, angle_in):
          self.theta = angle_in.data
     
     def calculate_pos(self, dist, theta):
          drone_pos = self.drone.get_local_position()
          h = drone_pos[2]
          ground = np.sqrt(dist**2 - h**2)
          print("ground distance: ",ground)
          target_x = ground * np.cos(theta)+ drone_pos[0]
          target_y = ground * np.sin(theta)+ drone_pos[1]
          print("targetx {} target y {}".format(
               round(target_x,2), round(target_y,2)))

          return [target_x , target_y]


          

if __name__ == "__main__":
     rospy.init_node("px4_operation_node", anonymous = False)
     ms = Mission()
     ms.act()
     rospy.spin()







