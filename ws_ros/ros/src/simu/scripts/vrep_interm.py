#!/usr/bin/env python3


# Import the necessary libraries
import rospy
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayLayout
from sensor_msgs.msg import Image # Image is the message type
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time
import numpy as np
  

class Blimp:

  def __init__(self):
    self.speed = [0., 0., 0.]
    self.yaw = 0.
    self.speed_yaw = 0.

    self.target = [0, 0]
    self.size_img = [0, 0]
    self.pos_cam = [0, 0, 0]
    self.pos_head = [0, 0, 0]

    self.forward_prop = 0
    self.side_top_prop = 0
    self.side_bottom_prop = 0
    self.alt_prop = 0

    self.Dvx = 0.5
    self.Dvy = 0.5
    self.Dpsi = 0.5
    self.m = 0.6

    self.dt = 0.1


  def actualise(self):
    self.speed[0] = self.speed[0] + (self.forward_prop - self.Dvx / self.m  * self.speed[0] + self.speed_yaw * self.speed[1]) * self.dt
    self.speed[1] +=  (self.side_top_prop - self.side_bottom_prop - self.Dvy / self.m  * self.speed[1] - self.speed_yaw * self.speed[0]) * self.dt
    self.speed_yaw += (self.side_top_prop + self.side_bottom_prop - self.Dpsi * self.speed_yaw)


def callback_img(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  frame = br.imgmsg_to_cv2(data)
  frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # # Find coordinates of target in the picture  
  # Red is tricky because its hue value is 0 as well as 360
  # if encountering problem, can use cv2.COLOR_BGR2RGB tu search for blue instead
  lower_red = np.array([0,50,50])
  upper_red = np.array([20,255,255])

  # Here we are defining range of bluecolor in HSV
  # This creates a mask of blue coloured 
  # objects found in the frame.
  mask = cv2.inRange(hsv, lower_red, upper_red)

  # The bitwise and of the frame and mask is done so 
  # that only the blue coloured objects are highlighted 
  # and stored in res
  res = cv2.bitwise_and(frame,frame, mask= mask)
  array = np.array(res[:, :, 2])
  tri = np.argwhere(array > 0)

  if tri.size != 0:
    target = [np.mean(tri[0]), np.mean(tri[1])]
  else : 
    target = [0, 0]
  blimp.target = target
  blimp.size_img = [len(res), len(res[0])]

  # Display image
  cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
  cv2.resizeWindow('camera', 600, 600)
  cv2.imshow("camera", frame)
  cv2.waitKey(1)

  
def callback_forward_prop(msg):
  blimp.forward_prop = msg.data

def callback_side_top_prop(msg):
  blimp.side_top_prop = msg.data

def callback_side_bottom_prop(msg):
  blimp.side_bottom_prop = msg.data

def callback_alt_prop(msg):
  blimp.alt_prop = msg.data

def callback_pos_camera(msg):
  blimp.pos_cam = [msg.x, msg.y, msg.z]

def callback_pos_head(msg):
  blimp.pos_head = [msg.x, msg.y, msg.z]
     
if __name__ == "__main__":
  blimp = Blimp()
  rospy.init_node('vrep_interm')
  subscriber_img = rospy.Subscriber("/image", Image, callback_img)

  subscriber_forward_prop = rospy.Subscriber("/forward_prop", Float64, callback_forward_prop)
  subscriber_side_top_prop = rospy.Subscriber("/side_top_prop", Float64, callback_side_top_prop)
  subscriber_side_bottom_prop = rospy.Subscriber("/side_bottom_prop", Float64, callback_side_bottom_prop)
  subscriber_alt_prop = rospy.Subscriber("/alt_prop", Float64, callback_alt_prop)

  subscriber_pos_camera = rospy.Subscriber("/camera", Vector3, callback_pos_camera)
  subscriber_pos_head = rospy.Subscriber("/head", Vector3, callback_pos_head)
  publisher_dist = rospy.Publisher("/dist_head", Float64)
  publisher_target = rospy.Publisher("/pos_target", Float64MultiArray)

  publisher_speed = rospy.Publisher("/speed_blimp", Float64MultiArray)
  r = rospy.Rate(10)

  while not rospy.is_shutdown():
      # actualise Blimp with Euler method
      blimp.actualise()

      # publish the speed for vrep
      speed_msg = Float64MultiArray()
      speed_msg.layout = MultiArrayLayout()
      speed_msg.data = blimp.speed + [blimp.speed_yaw]
      publisher_speed.publish(speed_msg)

      # publish distance of camera to head for control
      dist_msg = Float64()
      dist_msg.data = np.sqrt((blimp.pos_head[0] - blimp.pos_cam[0])**2 + (blimp.pos_head[1] - blimp.pos_cam[1])**2 + (blimp.pos_head[2] - blimp.pos_cam[2])**2)
      publisher_dist.publish(dist_msg)

      # publish position of head in the image
      target_msg = Float64MultiArray()
      target_msg.layout = MultiArrayLayout()
      target_msg.data = blimp.target + blimp.size_img
      publisher_target.publish(target_msg)

      r.sleep()
