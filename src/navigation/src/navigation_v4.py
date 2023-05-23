#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
import actionlib

import numpy as np
import math
import json
from misc import *

DISTANCE_TH = 0.5
WARM_UP_TIME = 10 # seconds
ANGULAR_TH = 1e-2
ROTATION_SPEED = 0.5 # rad/s
REACHED_TH = 0.5 # meters

class Navigation():
    
    def __init__(self, graph_path, frame_id, model_name, in_simulation=True, autorun=False, verbose=False):
        self.model_name = model_name
        self.frame_id = frame_id
        self._in_simulation = in_simulation
        self.verbose = verbose
        self.autorun = autorun

        # Inizializzare il grafo per la raggiungibilità dei waypoint
        V, E = read_graph(graph_path)
        self._graph, self._vertex_map = build_graph(V, E)
        
        # Inizializzare servizi ROS
        rospy.init_node('navigation')
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.on_shutdown(self.client.cancel_all_goals)
        
        self._goal = None
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.goal_reached)
        
        self._ccleaner = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # self._ccleaner_thread = rospy.Timer(rospy.Duration(4), self._costmap_cleaner)
          
    # def _costmap_cleaner(self, event):
    #     # source: https://answers.ros.org/question/278296/how-to-clear-local-cost-map/
    #     # rosservice call /move_base/clear_costmaps "{}"
    #     print('[NAV] Costmap cleaned')
    #     self._ccleaner()
    
    def goal_reached(self,amcl_pose:PoseWithCovarianceStamped):
        if self._goal:
            if np.linalg.norm(get_position_numpy(amcl_pose)-get_position_numpy(self._goal,to3D=True)) < DISTANCE_TH:
                self.client.cancel_all_goals()
                self._goal = None
                
                ## Rotation before end
                self._manage_pose(self._goal,to_pose2D(amcl_pose))                
                print('[NAV] GOAL REACHED')
    
    def get_command(self):
        msg : String = rospy.wait_for_message('/navigation/command', String)
        return msg.data
    
    def get_amcl_pose(self) -> Pose2D:
        return to_pose2D(rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped))
                
    def nearest_neighbor(self, point, neighbors):
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
            print(f"[NAV] dir_neighbors:\n{json.dumps(dir_neighbors,indent=3,cls=NpEncoder)}\n")
        
        if direction:
            return dir_neighbors[direction]
        
        return dir_neighbors
    
    def _movement_manager(self, movement:Twist, move_time, time_for_iteration=0.3):
        if self._in_simulation:
            self._pub_cmd_vel.publish(movement)
            rospy.sleep(move_time)
            self._pub_cmd_vel.publish(Twist())
        else:
            iterations = int(move_time/time_for_iteration) +1
            for i in range(iterations):
                self._pub_cmd_vel.publish(movement)
                rospy.sleep(time_for_iteration)
            self._pub_cmd_vel.publish(Twist())
            
    def _manage_pose(self, target_pose:Pose2D, current_pose:Pose2D=None):
        cmd = Twist()
        delta = target_pose.theta - current_pose.theta if current_pose else self._current_pose.theta
        if abs(delta) > np.pi:
            delta += -np.sign(delta)*np.pi*2

        time = 0 if abs(delta) < ANGULAR_TH else abs(delta / ROTATION_SPEED)
        
        cmd.angular.z = np.sign(delta) * ROTATION_SPEED
        self._movement_manager(cmd, time)
    
    def calibrate(self):
        self._ccleaner()
        cmd = Twist()
        cmd.angular.z = 2*np.pi/WARM_UP_TIME
        self._movement_manager(cmd, WARM_UP_TIME)        
    
    def set_goal(self, pose:Pose2D, verbose=False):
        if verbose:
            print('[NAV] New goal:')
            print(f"\tx: {pose.x:.3}")
            print(f"\ty: {pose.y:.3}")
            print(f"\ttheta: {math.degrees(pose.theta):.3}°")
        
        ## Rotation before goal
        self._manage_pose(pose)
        
        ## Setting goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = to_pose(pose)
        self.client.send_goal(goal)
        
        self._goal = pose
        
        # wait = self.client.wait_for_result()
        
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     # Result of executing the action
        #     return self.client.get_result()
    
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
        input('\nPress any key to START... ')
        
        while not rospy.is_shutdown() and current_cmd != 'STOP':
            
            print('---\n[NAV] command:', current_cmd)
            if not self.autorun:
                input('\nPress any key to continue... ')
            else:
                # waiting for settling
                rospy.sleep(1.5)
                
            self._current_pose = self.get_amcl_pose()
            print('[NAV] current pose:')
            print(f"\tx: {self._current_pose.x:.3}")
            print(f"\ty: {self._current_pose.y:.3}")
            print(f"\ttheta: {math.degrees(self._current_pose.theta):.3}°")
            
            # Reaching nearest waypoint
            wps = get_vertices_numpy(self._vertex_map)
            nearest_wp, distance = self.nearest_neighbor(self._current_pose,wps)
            print('[NAV] nearest wp:',nearest_wp,'at',f'{distance:.4f}')
            current_wp = tuple(nearest_wp)
            
            # Looking for reachable waypoints
            v = self._vertex_map[current_wp]
            adjs = [e.opposite(v).element() for e in self._graph.incident_edges(v)]
            
            # Find destination waypoint
            next_pos = self.directional_neighbors(self._current_pose,adjs,current_cmd)[0]
            theta = math.atan2(next_pos[1]-self._current_pose.y, next_pos[0]-self._current_pose.x)
            self.set_goal(to_pose2D(position=next_pos, orientation=(0,0, theta)), verbose=True)
            
            # Looking for next command
            min_trans = rospy.get_param('/move_base/DWAPlannerROS/min_vel_trans')
            rospy.set_param('/move_base/DWAPlannerROS/max_vel_trans',min_trans+2)
            current_cmd = self.get_command()
            rospy.set_param('/move_base/DWAPlannerROS/max_vel_trans',0.26)
            
        # if not self._in_simulation:
        #     self._ccleaner_thread.shutdown()

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
    
    
