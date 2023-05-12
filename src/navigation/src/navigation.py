#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionResult

from ppoint import PPoint
import numpy as np
from misc import *
from src.navigation.src.graphs.graph_utilis import *
from src.navigation.src.graphs.graph import Graph

DISTANCE_TH = 0.5
WARM_UP_TIME = 10 # seconds

class Navigation():
    
    def __init__(self, graph_path, frame_id, model_name, in_simulation=True, autorun=False, verbose=False):
        self.model_name = model_name
        self.frame_id = frame_id
        self.trajecotry = list()
        self._in_simulation = in_simulation
        self.verbose = verbose
        self.autorun = autorun

        # Inizializzare il grafo per la raggiungibilità dei waypoint
        V, E = read_graph(graph_path)
        self._graph, self._vertex_map = build_graph(V, E)
        
        # Inizializzare servizi ROS
        rospy.init_node('navigation')
        self._pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=3)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self._sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._update_trajectory)
        
    def _update_trajectory(self, pose:PoseWithCovarianceStamped):
        self.trajecotry.append(pose)
                
    def nearest_neighbor(self, ego_position, neighbors):
        neighbors = np.array(neighbors)
        dist = np.linalg.norm(neighbors-ego_position, axis=1)
        idx = np.argmin(dist)
        return neighbors[idx], dist[idx]
    
    def directional_neighbors(self, ego_pose:PPoint, neighbors, direction:str=None):
        dir_neighbors = {'left':[], 'right':[], 'front':[], 'back':[]}
        for vx, vy in neighbors:
            neighbor = np.array([vx,vy,0])
            v = ego_pose.get_position_numpy() - neighbor
            u = np.array([np.cos(ego_pose._orientation._yaw), np.sin(ego_pose._orientation._yaw),0])

            cross = np.cross(u,v)[2] # vector along z of the vector product
            inner = np.inner(u,v)    # scalar product
            
            if abs(cross) > abs(inner):
                if cross > 0:
                    dir_neighbors['left'].append((vx,vy))
                else :
                    dir_neighbors['right'].append((vx,vy))
            else:
                if inner >= 0:
                    dir_neighbors['front'].append((vx,vy))
                else:
                    dir_neighbors['back'].append((vx,vy))
        
        if direction:
            return dir_neighbors[direction]
        
        return dir_neighbors
    
    def calibrate(self):
        print("[NAV] Calibration phase...",end='')
        cmd = Twist()
        cmd.angular.z = 2*np.pi/WARM_UP_TIME
        self._pub_cmd_vel.publish(cmd)
        rospy.sleep(WARM_UP_TIME)
        cmd.angular.z = 0
        self._pub_cmd_vel.publish(cmd)
        print("DONE")
    
    def set_goal(self, goal_pose:PPoint):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose = goal_pose.to_pose()
        
        self._pub_goal.publish(msg)
        
        result : MoveBaseActionResult = rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
        return result
    
    def start(self):
        
        ## Positionig on starting point
        if self._in_simulation:
            print("## Insert starting pose")
            str_position = str2tuple(input("position (x,y,z): "))
            str_orientation = str2tuple(input("orientation (roll, pitch, yaw): "))          
            tmp = PPoint('starting_pose',str_position,str_orientation)
            jump_to(self.model_name, tmp, hard_reset=False)
            print("##\n")
        
        print("[NAV] Waiting for RViz 2D Pose Estimate")
        input('Press any key to continue...')
        
        ## Calibration phase
        self.calibrate()
        
        ## Reaching nearest waypoint
        tmp : PoseWithCovarianceStamped = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        starting_pose = PPoint.from_pose_to_ppoint(tmp.pose.pose)
        starting_pose.id = 'starting pose'
        print('\n>> starting pose:',starting_pose)
        wps = get_vertices_numpy(self._vertex_map)
        nearest_wp, distance = self.nearest_neighbor(starting_pose.get_position_numpy(),wps)
        print('>> nearest wp:',nearest_wp,'at',f'{distance:.4f}')
        if distance > DISTANCE_TH:
            # if nearest_wp is too far reach it
            print('[NAV] nearest wp far, reaching...')
            self.set_goal(PPoint(position=nearest_wp))
        current_wp = nearest_wp
        
        current_cmd = 'STRAIGHT ON'
        
        # while not rospy.is_shutdown():
            
        #     if not self.autorun:
        #         input('-\nPress any key to continue... ')
        #     else:
        #         # waiting for settling
        #         rospy.sleep(3)
            
            

if __name__ == '__main__':
    import os
    from pathlib import Path

    m = Navigation(graph_path=      Path(os.environ['WS_DIR']) / 'src/navigation/landmarks/reach.txt',
                   frame_id=        'map',
                   model_name=      os.environ['TURTLEBOT_MODEL'],
                   in_simulation=   bool(rospy.get_param('simulation',0)),
                   autorun=         False,
                   verbose=         False)
    try:
        m.start()
    except rospy.ROSInterruptException:
        pass

    