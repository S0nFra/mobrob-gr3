#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

# numpy_ros sources:
# https://pypi.org/project/numpy-ros/
# https://docs.ros.org/en/jade/api/ros_numpy/html/namespaceros__numpy_1_1geometry.html
# from numpy_ros import to_numpy

from ppoint import PPoint
import numpy as np
import json

class Navigation():
    
    def __init__(self, trajectory_path, frame_id, model_name, autorun=False, verbose=False):
        self.model_name = model_name
        self.frame_id = frame_id
        self.verbose = verbose
        self.autorun = autorun

        self.trajectory = []
        trajectory_json = json.load(open(trajectory_path))
        for point in trajectory_json:
            tmp = trajectory_json[str(point)]['pose']
            waypoint = PPoint(str(point), list(tmp['position'].values()), list(tmp['orientation'].values()))
            self.trajectory.append(waypoint)
                        
            if self.verbose:
                print(waypoint,'\n')
  
    def start(self):
        rospy.init_node('navigation')
        pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
        
        # jump_to(self.model_name, self.trajectory[0], hard_reset=True) # mi posiziono nel primo punto della traiettoria
        
        cp : PPoint = self.trajectory[0]  # cp - current_point
        dp : PPoint                       # dp - destination_point
        for dp in self.trajectory[1:]:
            
            if not self.autorun:
                input('-\nPress enter to continue... ')
            else:
                # waiting for settling
                rospy.sleep(3)
            
            print(f'> Moving to point {dp.id}')
            if self.verbose:
                print(dp,'\n')
            
            msg = PoseStamped()
            msg.header.frame_id = self.frame_id
            msg.pose = dp.to_pose()
            pub_goal.publish(msg)
            
            result : MoveBaseActionResult = rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
            # print(result)

if __name__ == '__main__':
    import os
    from pathlib import Path

    m = Navigation(trajectory_path=  Path(os.environ['WS_DIR']) / 'src/navigation/landmarks/traj.json',
                   frame_id=         'map',
                   model_name=       os.environ['TURTLEBOT_MODEL'],
                   autorun=          False,
                   verbose=          True)   
    try:
        m.start()
    except rospy.ROSInterruptException:
        pass

    