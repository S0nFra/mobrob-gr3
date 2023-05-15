#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult

import numpy as np
import math
import json
from misc import *
from graph import Graph

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
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        # self._sub_amcl = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._update_trajectory)
        
    # def _update_trajectory(self, pose:PoseWithCovarianceStamped):
    #     self.trajecotry.append(pose)
    
    def get_command(self):
        msg : String = rospy.wait_for_message('/navigation/command', String)
        return msg.data
    
    def get_amcl_pose(self) -> Pose2D:
        return to_pose2D(rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped))
                
    def nearest_neighbor(self, point, neighbors):
        # neighbors = np.array(neighbors)
        dist = np.linalg.norm(neighbors-get_position_numpy(point), axis=1)
        idx = np.argmin(dist)
        return neighbors[idx], dist[idx]
    
    def directional_neighbors(self, point:Pose2D, neighbors, direction:str=None, verbose=False):
        dir_neighbors = {'LEFT':[], 'RIGHT':[], 'STRAIGHT ON':[], 'GO BACK':[]}
        
        neighbors = np.array(neighbors)
        if neighbors.shape[1] < 3:
            neighbors = np.c_[neighbors,np.zeros(neighbors.shape[0])]
        
        for neighbor in neighbors:
            v = neighbor - get_position_numpy(point,to3D=True)
            u = np.array([np.cos(point.theta), np.sin(point.theta),0])
            cross = np.cross(u,v)[2] # vector along z of the vector product
            inner = np.inner(u,v)    # scalar product
            
            if abs(cross) > abs(inner):
                if cross > 0:
                    dir_neighbors['LEFT'].append(neighbor)
                else :
                    dir_neighbors['RIGHT'].append(neighbor)
            else:
                if inner >= 0:
                    dir_neighbors['STRAIGHT ON'].append(neighbor)
                else:
                    dir_neighbors['GO BACK'].append(neighbor)
        
        if verbose:
            class NpEncoder(json.JSONEncoder):
                def default(self, obj):
                    if isinstance(obj, np.integer):
                        return int(obj)
                    if isinstance(obj, np.floating):
                        return float(obj)
                    if isinstance(obj, np.ndarray):
                        return obj.tolist()
                    return json.JSONEncoder.default(self, obj)
            print(f"[NAV] dir_neighbors:\n{json.dumps(dir_neighbors,indent=3,cls=NpEncoder)}\n")
        
        if direction:
            return dir_neighbors[direction]
        
        return dir_neighbors
    
    def calibrate(self):
        cmd = Twist()
        cmd.angular.z = 2*np.pi/WARM_UP_TIME
        self._pub_cmd_vel.publish(cmd)
        rospy.sleep(WARM_UP_TIME)
        cmd.angular.z = 0
        self._pub_cmd_vel.publish(cmd)
    
    def set_goal(self, pose:Pose2D, verbose=False):
        
        if verbose:
            print('[NAV] New goal:\n',pose,'\n',sep='')
        
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose = to_pose(pose)
        
        self._pub_goal.publish(msg)
        
        result : MoveBaseActionResult = rospy.wait_for_message('/move_base/result', MoveBaseActionResult)
        return result
    
    def start(self):
        
        ## Positionig on starting point
        if self._in_simulation:
            print("## Insert starting pose")
            str_position = str2list(input("position (x,y): "))
            str_orientation = math.radians(float(input("orientation (yaw°): ")))
            jump_to(self.model_name, to_pose(position=str_position, orientation=str_orientation), hard_reset=False)
            print("##\n")
        
        print("[NAV] Waiting for RViz 2D Pose Estimate")
        input('Press any key to continue...')
        
        ## Calibration phase
        print("[NAV] Calibration phase... ",end='')
        self.calibrate()
        print("DONE")
        
        current_cmd = 'STRAIGHT ON'
        
        while not rospy.is_shutdown() and current_cmd != 'STOP':
            
            print('-\n[NAV] command:', current_cmd)
            
            if not self.autorun:
                input('Press any key to continue... ')
            else:
                # waiting for settling
                rospy.sleep(3)
                
            current_pose = self.get_amcl_pose()
            print('[NAV] current pose:\n',current_pose,'\n')
            
            # Reaching nearest waypoint
            wps = get_vertices_numpy(self._vertex_map)
            nearest_wp, distance = self.nearest_neighbor(current_pose,wps)
            print('[NAV] nearest wp:',nearest_wp,'at',f'{distance:.4f}')
            current_wp = tuple(nearest_wp)
            self.trajecotry.append(current_wp)
            
            # Looking for reachable waypoints
            v = self._vertex_map[current_wp]
            adjs = [e.opposite(v).element() for e in self._graph.incident_edges(v)]
            print(adjs)
            # Find destination waypoint
            next_pos = self.directional_neighbors(current_pose,adjs,current_cmd)[0]
            theta = math.atan2(next_pos[1]-current_pose.y, next_pos[0]-current_pose.x)
            self.set_goal(to_pose2D(position=next_pos, orientation=(0,0,theta)), verbose=True)
            
            # Looking for next command
            current_cmd = self.get_command()


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

    