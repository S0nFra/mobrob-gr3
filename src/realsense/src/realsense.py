#!/bin/python3

import rospy
from pathlib import Path
import cv2 as cv
import cv_bridge
from sensor_msgs.msg import Image, CompressedImage
from threading import Thread

class Node:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME, anonymous=True)
        self.camera_id = rospy.get_param("~camera_id")
        self.cap = self.open_stream()
        self.set_camera()
        self.print_camera_info()
        self.bridge = cv_bridge.CvBridge()
        self.img_pub = rospy.Publisher("image", CompressedImage, queue_size=1)
        self.rate = rospy.Rate(rospy.get_param("/camera/fps_publish"))
        self.frame = None
        Thread(target=self.publish_frame_cb, daemon=True).start()
    
    def open_stream(self):
        '''
        This code is opening a camera stream using OpenCV's `cv.VideoCapture()` method with the camera ID
        specified in the `self.camera_id` variable. If the camera fails to open, it shuts down the ROS node. 
        '''
        cap = cv.VideoCapture(self.camera_id)
        if not cap.isOpened():
            rospy.logerr(f"Cannot open camera {self.camera_id}")
            rospy.signal_shutdown("Camera error")
        rospy.loginfo("Camera opened")
        return cap

    def set_camera(self):
        '''
        These lines of code are setting the properties of the camera stream using OpenCV's
        cv.VideoCapture()` method. Specifically, it is setting the frames per second (FPS) of the camera
        stream to the value specified in the ROS parameter `/camera/fps_capture`, and setting the frame
        width and height to the values specified in the ROS parameters `/camera/width` and `/camera/height`,
        respectively.
        '''
        self.cap.set(cv.CAP_PROP_FPS, rospy.get_param("/camera/fps_capture"))
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, rospy.get_param("/camera/width"))    
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, rospy.get_param("/camera/height"))   

    def print_camera_info(self):
        '''
        These lines of code are retrieving the width, height, and frames per second (FPS) of the
        camera stream using OpenCV's `cv.VideoCapture()` method. The values are then logged using
        `rospy.loginfo()`.
        '''
        width = self.cap.get(cv.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv.CAP_PROP_FRAME_HEIGHT)
        fps = self.cap.get(cv.CAP_PROP_FPS)
        rospy.loginfo(f"Frame size width x height: {int(width)}x{int(height)}@{fps} for camera: {self.camera_id}")
    
    def publish_frame(self, frame):
        '''
        These lines of code are converting the OpenCV `frame` image to a compressed image message using the
        `cv_bridge` library's `cv2_to_compressed_imgmsg()` method. The compressed image message is then
        published to the ROS topic `"image"` using the `self.img_pub.publish()` method. This allows the
        image to be transmitted over the ROS network with reduced bandwidth usage.
        '''
        img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, 'jpg')
        self.img_pub.publish(img_msg)
    
    def publish_frame_cb(self):
        '''
        This code is running in a separate thread and continuously checking if the ROS node is still running
        using `rospy.is_shutdown()`. If the node is still running and there is a new frame available in
        `self.frame`, it publishes the frame as a compressed image message to the ROS topic `"image"` using
        the `self.publish_frame()` method. It then sleeps for a duration of time specified by `self.rate`
        before checking again. This ensures that frames are continuously published to the ROS network at a
        consistent rate.
        '''
        while not rospy.is_shutdown():
            if self.frame is not None:
                self.publish_frame(self.frame)
            self.rate.sleep()
        
    def loop(self):
        '''
        This code is continuously reading frames from the camera stream using OpenCV's `cv.VideoCapture()`
        method in a loop. It checks if the ROS node is still running using `rospy.is_shutdown()`. If the
        node is still running and a new frame is successfully acquired, it sets the `self.frame` variable to
        the new frame. If a frame is not acquired, it logs an error message using `rospy.logerr()`. This
        loop ensures that frames are continuously read from the camera stream and stored in `self.frame` for
        publishing to the ROS network.
        '''
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr(f"Frame not acquired for camera {self.camera_id}")
            self.frame = frame
    
if __name__ == "__main__":
    NODE_NAME = "realsense_node"
    node = Node()
    node.loop()
