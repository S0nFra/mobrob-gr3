#!/usr/bin/python3

from sensor_msgs.msg import CompressedImage, Image, String
import message_filters
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class Camera():

    __slots__ = '_pub', '_qr'

    def __init__(self):
        rospy.init_node('camera_listener')
        self._pub = rospy.Publisher('/navigation/command', String, queue_size=3)
        self._qr = cv2.QRCodeDetector()
    
    def start(self):
        rospy.Subscriber('/camera/lx/image', CompressedImage, self._scan_images_lx)
        rospy.Subscriber('/camera/rx/image', CompressedImage, self._scan_images_rx)
        rospy.spin()
        #ts = message_filters.TimeSynchronizer([sub_rx, sub_lx], 10)
        #ts.registerCallback(self._scan_images)

    def _scan_images_lx(self, frame):
        bridge = CvBridge()

        try:
            image = bridge.compressed_imgmsg_to_cv2(frame, "jpg")
            cv2.imshow('Camera LX', frame)
            cv2.waitkey(1)
        except CvBridgeError as e:
            print('ERROR: scan images')
        
        command, _ , _ = self._qr.detectAndDecodeCurved(image)
        
        print('Next command from lx: ', command)

        if command: self._pub.publish(command)

        #cv2.destroyAllWindows()
    
    def _scan_images_rx(self, frame):
        bridge = CvBridge()

        try:
            image = bridge.compressed_imgmsg_to_cv2(frame, "jpg")
            cv2.imshow('Camera RX', frame)
            cv2.waitkey(1)
        except CvBridgeError as e:
            print('ERROR: scan images')
        
        command, _ , _ = self._qr.detectAndDecodeCurved(image)
        
        print('Next command from rx: ', command)
        
        if command: self._pub.publish(command)
        
        
if __name__ == '__main__':
    cam = Camera()
    try:
        cam.start()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
    
    

