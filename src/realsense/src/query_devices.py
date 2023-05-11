#!/bin/python3

import os
import re
import rospy
import cv2 as cv
import subprocess

class Node:
    def __init__(self) -> None:
        rospy.init_node(NODE_NAME)
        pass
    
    def main(self):
        _, _, files = next(os.walk("/dev/"))
        indexes = []
        for device in files:
            for device_video in re.findall("video[0-9]+", device):
                index = device_video.split("video")[1]
                cap = cv.VideoCapture(int(index))
                if not cap.isOpened():
                    continue
                cap.release()
                cmd =  f"v4l2-ctl --device /dev/video{index} --all"
                result = subprocess.run(cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
                info = result.stdout
                if "RealSense" in info and "YUYV" in info:
                    indexes.append(index)
                
        rospy.loginfo(f"Valid indexes: {indexes}")
        rospy.set_param("indexes", indexes)

if __name__ == "__main__":
    NODE_NAME = "query_device_node"
    node = Node()
    node.main()