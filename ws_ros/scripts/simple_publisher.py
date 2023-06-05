#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_path
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import numpy as np  # numpy - manipulate the packet data returned by depthai
import cv2  # opencv - display the video stream
import depthai  # depthai - access the camera and its data packets
import blobconverter  # blobconverter - compile and download MyriadX neural network blobs


class ImagePublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_publisher')
            
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
            
        # We will publish a message every 0.01 seconds
        timer_period = 0.01  # seconds
            
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
                
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # print("This is obviously bestest ROS2 node ever")

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """

        self.publisher_.publish(self.br.cv2_to_imgmsg(self.frame))
    
        # Display the message on the console
        self.get_logger().info('Publishing video frame')



  
def main(args=None):

    #Code for luxonis
    pipeline = depthai.Pipeline()

    cam_rgb = pipeline.create(depthai.node.ColorCamera)
    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setInterleaved(False)

    detection_nn = pipeline.create(depthai.node.MobileNetDetectionNetwork)
    # Set path of the blob (NN model). We will use blobconverter to convert&download the model
    # detection_nn.setBlobPath("/path/to/model.blob")
    detection_nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6))
    detection_nn.setConfidenceThreshold(0.5)

    cam_rgb.preview.link(detection_nn.input)

    xout_rgb = pipeline.create(depthai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    xout_nn = pipeline.create(depthai.node.XLinkOut)
    xout_nn.setStreamName("nn")
    detection_nn.out.link(xout_nn.input)

    device = depthai.Device(pipeline, usb2Mode=True)
    # device = depthai.Device(pipeline)
    q_rgb = device.getOutputQueue("rgb")
    q_nn = device.getOutputQueue("nn")
    frame = None
    detections = []

    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher()

    frame = None
    while True:
        in_rgb = q_rgb.tryGet()
        in_nn = q_nn.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()
        if in_nn is not None:
            detections = in_nn.detections
        if frame is not None:
            for detection in detections:
                bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
            cv2.imshow("preview", frame)
            if cv2.waitKey(1) == ord('q'):
                break
            # Spin the node so the callback function is called.
            image_publisher.frame = frame
            rclpy.spin_once(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    # print("I will see you beautiful face in the next topic")

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
  main()
