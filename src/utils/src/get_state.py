#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
import os

def callback(data:ModelStates):
    print(data.name)
    print(data.pose)
    print('---')

def listener():

    rospy.init_node('listener', anonymous=True)
    srv = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)

    while not rospy.is_shutdown():
        resp = srv.call(GetModelStateRequest(os.environ['TURTLEBOT_MODEL'],'world'))
        resp : GetModelStateResponse
        # print(resp.header)
        print(resp.pose,'\n---')
    
if __name__ == '__main__':
    listener()