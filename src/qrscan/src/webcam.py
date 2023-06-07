#!/usr/bin/python3

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import threading
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Webcam():

    def __init__(self, name:str, topic:str):
        self.name = name
        self.topic = topic
        self.video = None
        self.bridge = CvBridge()
    
    def start(self):
        rospy.init_node(self.name)
        self._pub = rospy.Publisher(self.topic, CompressedImage, queue_size=1)
        self.video = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            self._take()
        
        self.video.release()
        
    def _take(self):
        _ , frame = self.video.read()
    
        try:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame, 'jpg')
            self._pub.publish(img_msg)
        except CvBridgeError as e:
            print('ERROR: take images')

if __name__ == '__main__':
    
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("--name", "-n", default='camera')
    parser.add_option("--topic", "-t", default='topic')    

    (options, args) = parser.parse_args()
    
    cam = Webcam(name=options.name,
                 topic=options.topic
    )

    print(options)

    try:
        cam.start()
    except rospy.ROSInterruptException:
        pass
    
    

