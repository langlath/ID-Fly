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
    self.pos = [0., 0., 0.]
    self.speed = [0., 0., 0.]
    self.yaw = 0.
    self.speed_yaw = 0.

    self.target = [0, 0]

    self.forward_prop = 0
    self.side_top_prop = 0
    self.side_bottom_prop = 0
    self.alt_prop = 0

    self.Dvx = 0.5
    self.Dvy = 0.5
    self.Dpsi = 0.5
    self.m = 0.6

    self.dt = 0.1

  @property
  def x(self):
    return self.pos[0]

  @x.setter
  def x(self, new_x):
    self.pos[0] = new_x

  @property
  def y(self):
    return self.pos[1]
  
  @y.setter
  def y(self, new_y):
    self.pos[0] = new_y

  
  @property
  def z(self):
    return self.pos[2]
  
  @z.setter
  def z(self, new_z):
    self.pos[0] = new_z


  def actualise(self):
    self.x += self.speed[0] * self.dt
    self.y += self.speed[1] * self.dt
    self.z += self.speed[2] * self.dt
    self.yaw += self.speed_yaw * self.dt

    # print("self.speed_x avant", self.speed[0])
    self.speed[0] = self.speed[0] + (self.forward_prop - self.Dvx / self.m  * self.speed[0] + self.speed_yaw * self.speed[1]) * self.dt
    # print((self.forward_prop - self.Dvx / self.m  * self.speed[0] + self.speed_yaw * self.speed[1]) * self.dt)
    # print("self.speed_x après", self.speed[0])
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
  # lis_x_head = []
  # lis_y_head = []
  # for i in range(len(frame)):
  #   for j in range(len(frame[0])):
  #     if frame[i][j][0] >= 100 and frame[i][j][0] > 2 * max(frame[i][j][1], frame[i][j][2]):
  #       lis_x_head.append(i)
  #       lis_y_head.append(j)
  # x_head = np.mean(np.array(lis_x_head))
  # y_head = np.mean(np.array(lis_y_head))
  # print(x_head, y_head)
  # print(len(frame), len(frame[0]))
  
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

  if tri.size != np.array([]):
    target = (np.mean(tri[0]), np.mean(tri[1]))
  else : 
    target = (0, 0)
  print(target)
  # print(array)

  # Display image
  cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
  cv2.resizeWindow('camera', 600, 600)
  cv2.imshow("camera", frame)
  cv2.waitKey(1)

def callback_pos(msg):
    blimp.x = msg.x
    blimp.y = msg.y
    blimp.z = msg.z
  
def callback_forward_prop(msg):
  blimp.forward_prop = msg.data
  # print(blimp.speed)
     
if __name__ == "__main__":
  blimp = Blimp()
  rospy.init_node('vrep_interm')
  subscriber_img = rospy.Subscriber("/image", Image, callback_img)
  subscriber_pos = rospy.Subscriber("/blimp", Vector3, callback_pos)
  subscriber_forward_prop = rospy.Subscriber("/forward_prop", Float64, callback_forward_prop)
  publisher_speed = rospy.Publisher("/speed_blimp", Float64MultiArray)
  r = rospy.Rate(10)
  while not rospy.is_shutdown():
      blimp.actualise()
      msg = Float64MultiArray()
      msg.layout = MultiArrayLayout()
      msg.data = blimp.speed
      publisher_speed.publish(msg)
      r.sleep()
