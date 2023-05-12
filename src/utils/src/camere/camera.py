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

class Camera():

    def __init__(self, name:str, topic:str, save_path=None, show=False):

        self.name = name
        self.topic = topic
        self.show = show
        self.save_path = save_path
        self.frame_cnt = 0
        self._pub = rospy.Publisher('/navigation/command', String, queue_size=3)
        self._qr = cv2.QRCodeDetector()
        self.bridge = CvBridge()
    
    def start(self):
        rospy.init_node(self.name)
        rospy.Subscriber(self.topic, CompressedImage, self._scan_images)
        rospy.spin()
        
    def _scan_images(self, frame:CompressedImage):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(frame)
            if self.show:
                cv2.imshow(self.name, image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print('ERROR lx: scan images')
            
        if self.save_path:
            cv2.imwrite(self.save_path+f'/{self.frame_cnt}.jpg',image)
            self.frame_cnt += 1
        
        command, _ , _ = self._qr.detectAndDecodeCurved(image)
        
        if command: 
            print('Next command from lx: ', command)
            self._pub.publish(command)

if __name__ == '__main__':
    
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("--name", "-n", default='camera')
    parser.add_option("--topic", "-t", default='topic')
    parser.add_option("--save", default=None)
    parser.add_option("--show", "-s", default=False)    
    (options, args) = parser.parse_args()
    
    cam = Camera(name=options.name,
                 topic=options.topic,
                 save_path=options.save,
                 show=options.show)
    print(options)

    try:
        cam.start()
    except rospy.ROSInterruptException:
        pass
    
    

