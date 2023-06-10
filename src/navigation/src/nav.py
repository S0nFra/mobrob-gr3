#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped, PoseWithCovarianceStamped, Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from navigation.srv import *
import dynamic_reconfigure.client
import actionlib
from threading import Event

from enum import Enum
import numpy as np
import math
import json
from misc import *

WARM_UP_TIME = 10     # seconds
ANGULAR_TH = 1e-2
ROTATION_SPEED = 0.5  # rad/s
REACHED_TH = 1.5      # meters
REACHED_TH_STOP = 0.1 # meters
SLOW_SPEED = 0.20
FAST_SPEED = 0.26
RESEARCH_ATTEMPTS = 2 # integers multiples of 2

class Command(str,Enum):
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    STRAIGHT_ON = 'STRAIGHT ON'
    GO_BACK = 'GO BACK'
    STOP = 'STOP'
    REPOSITION = 'REP'

class Navigation():
    
    def __init__(self, graph_path, frame_id, model_name, in_simulation=True, autorun=False, verbose=False):
        self.model_name = model_name
        self.frame_id = frame_id
        self._in_simulation = in_simulation
        self.verbose = verbose
        self.autorun = autorun
        
        self._goal = None
        self.current_cmd = None

        self._current_pose = None
        self._last_waypoint = None
        self._ref_covariance = [0.214366998116555, 0.002494660950836193, 0.0, 0.0, 0.0, 0.0, 0.002494660950836189, 0.15795109038591174, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05642449211155298]
        self._target_wp = None
        self._flag = False
        self._iteration = 3
        self._set_goal_completed = Event()

        # Initialize the waypoint reachability graph
        V, E = read_graph(graph_path)
        self._graph, self._vertex_map = build_graph(V, E)
        
        # Initialize ROS services
        rospy.init_node('navigation')
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        rospy.Subscriber("/navigation/command", String, self.get_command)
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.on_shutdown(self.move_base_client.cancel_all_goals)
        
        self.reconfigure_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
        self.reconfigure_client.update_configuration({"xy_goal_tolerance":REACHED_TH, "max_vel_trans":SLOW_SPEED})
        
        self._pub_rviz_pose = rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=1)
        
        self._ccleaner = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        rospy.Service('/cancel_goal',CancelGoal,self.cancel_goal)

    def cancel_goal(self, _):
        self.current_cmd = Command.REPOSITION
        self._flag = False
        self._set_goal_completed.set() if not self._set_goal_completed.is_set() else None
        self.move_base_client.cancel_all_goals()
        return CancelGoalResponse("[ACK]")
       
    def get_command(self, command:String):
        self.reconfigure_client.update_configuration({"max_vel_trans":FAST_SPEED})
        self.current_cmd = command.data
        
        if self.current_cmd == Command.STOP:
            self.reconfigure_client.update_configuration({"xy_goal_tolerance":REACHED_TH_STOP})
        
        if self._flag and self._iteration == 0:
            self._flag = False
            self.move_base_client.cancel_all_goals()
            rospy.sleep(1)
            self.set_goal(self._target_wp, verbose=True)
            self._set_goal_completed.set()

    def get_amcl_pose(self) -> Pose2D:
        return to_pose2D(rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped))
                
    def nearest_neighbor(self, point, neighbors):
        dist = np.linalg.norm(neighbors-get_position_numpy(point), axis=1)
        idx = np.argmin(dist)
        return neighbors[idx], dist[idx]
    
    def directional_neighbors(self, point:Pose2D, neighbors, direction:str=None, verbose=False):
        dir_neighbors = {Command.LEFT:[], Command.RIGHT:[], Command.STRAIGHT_ON:[], Command.GO_BACK:[]}
        
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
                    dir_neighbors[Command.LEFT].append(neighbor)
                else :
                    dir_neighbors[Command.RIGHT].append(neighbor)
            else:
                if inner >= 0:
                    dir_neighbors[Command.STRAIGHT_ON].append(neighbor)
                else:
                    dir_neighbors[Command.GO_BACK].append(neighbor)
        
        if verbose:
            print(f"[NAV] dir_neighbors:\n{json.dumps(dir_neighbors,indent=3,cls=NpEncoder)}\n")
        
        res = None
        try:
            res = dir_neighbors[direction]
        except KeyError:
            print("[NAV] No correct direction:",direction)

        if not res:
            print("[NAV] No points in direction",direction)
            return None
                
        return res[0]
    
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
            
    def _manage_pose(self, target_pose:Pose2D):
        cmd = Twist()
        delta = target_pose.theta - self._current_pose.theta
        if abs(delta) > np.pi:
            delta += -np.sign(delta)*np.pi*2

        time = 0 if abs(delta) < ANGULAR_TH else abs(delta / ROTATION_SPEED)
        
        cmd.angular.z = np.sign(delta) * ROTATION_SPEED
        self._movement_manager(cmd, time)
    
    def calibrate(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
        self._ccleaner()
        cmd = Twist()
        cmd.angular.z = 2*np.pi/WARM_UP_TIME
        self._movement_manager(cmd, WARM_UP_TIME)        
    
    def set_goal(self, pose:Pose2D, verbose=False):
        if verbose:
            print('[NAV] new goal:')
            print(f"\tx: {pose.x:.3}")
            print(f"\ty: {pose.y:.3}")
            print(f"\ttheta: {math.degrees(pose.theta):.3}°")
        
        ## Rotation before goal
        self._manage_pose(pose)
        rospy.sleep(0.5)
        
        ## Setting goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = to_pose(pose)
        self._goal = pose
        self._target_wp = pose if not self._cmd_force else self._target_wp

        self.move_base_client.send_goal(goal)
        
        wait = self.move_base_client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            pprint("[NAV] Goal reached",bcolors.BOLD)
            rospy.wait_for_service('/move_base/clear_costmaps')
            self._ccleaner()
            return self.move_base_client.get_result()
        
    def robot_repositioning_manger(self):
        print('[NAV] expected waypoint:',self._last_waypoint)
        
        if self._in_simulation:
            print("## Insert pose")
            pos = str2list(input("position (x,y): "))
            ori = math.radians(float(input("orientation (yaw°): ")))
            jump_to(self.model_name, to_pose(position=pos, orientation=ori), hard_reset=False)
            print("##\n")
        else:
            input('[NAV] waiting for repositioning. Press any key to conitune...')
        
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.pose.position.x = self._last_waypoint.x
        pose.pose.pose.position.y = self._last_waypoint.y
        print('ANGOLO:', math.degrees(self._last_waypoint.theta))
        tmp = quaternion_from_euler(0,0,self._last_waypoint.theta)
        pose.pose.pose.orientation = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        pose.pose.covariance = self._ref_covariance
        self._pub_rviz_pose.publish(pose)
        rospy.sleep(3) 
        
        print("[NAV] Calibration phase... ",end='')
        self.calibrate()
        print("DONE")
        
        self.current_cmd = Command.STRAIGHT_ON
        input('[NAV] Ready to restart. Press any key to contiune...')
        
    def execute_command(self, command_force=None):
        
        command = command_force if command_force else self.current_cmd
        self._cmd_force = True if command_force else False

        self._current_pose = self.get_amcl_pose()
        print('[NAV] current pose:')
        print(f"\tx: {self._current_pose.x:.3}")
        print(f"\ty: {self._current_pose.y:.3}")
        print(f"\ttheta: {math.degrees(self._current_pose.theta):.3}°")
        
        # Reaching nearest waypoint
        wps = get_vertices_numpy(self._vertex_map)
        nearest_wp, distance = self.nearest_neighbor(self._current_pose,wps)
        print('[NAV] nearest wp:',nearest_wp,'at',f'{distance:.4f} meters')
        current_wp = tuple(nearest_wp)
        
        # Looking for reachable waypoints
        v = self._vertex_map[current_wp]
        reachable_wps = [e.opposite(v).element() for e in self._graph.incident_edges(v)]
        
        # Find destination waypoint
        next_pos = self.directional_neighbors(self._current_pose,reachable_wps,command)
        
        if not command_force:
            self.current_cmd = None
        
        if next_pos is not None:
            theta = math.atan2(next_pos[1]-nearest_wp[1], next_pos[0]-nearest_wp[0])
            self.set_goal(to_pose2D(position=next_pos, orientation=(0,0, theta)), verbose=True)
        else:
            theta = None
            pprint("[NAV] required repositioning!",bcolors.YELLOW)
            self.robot_repositioning_manger()
        
        if not command_force:
            self._last_waypoint = to_pose2D(position=current_wp, orientation=(0,0, theta))
    
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
        
        self.current_cmd = Command.STRAIGHT_ON
        input('\nPress any key to START... ')
        
        while not rospy.is_shutdown() and self.current_cmd != Command.STOP:
            
            if not self.current_cmd:
                pprint('[NAV] Command not found',bcolors.YELLOW)
                self._flag = True
                for i in range(RESEARCH_ATTEMPTS):
                    print(f'[NAV] Looking for command. Try {i+1}/{RESEARCH_ATTEMPTS}')
                    self._iteration = i
                    self.execute_command(command_force=Command.GO_BACK)
                    if not self._flag:
                        self._set_goal_completed.wait()
                        self._set_goal_completed.clear()
                        break
                if self.current_cmd == Command.STOP:
                    continue
                self.reconfigure_client.update_configuration({"max_vel_trans":SLOW_SPEED})
            
            print('---\n[NAV] command:', self.current_cmd)
            if not self.autorun:
                input('\nPress any key to continue... ')
            else:
                # waiting for settling
                rospy.sleep(1.5)

            self.execute_command()

            # Go slow to detect command            
            self.reconfigure_client.update_configuration({"max_vel_trans":SLOW_SPEED})
            
        pprint("""[NAV]
  ______ _   _ _____  
 |  ____| \ | |  __ \ 
 | |__  |  \| | |  | |
 |  __| | . ` | |  | |
 | |____| |\  | |__| |
 |______|_| \_|_____/  
         
""",bcolors.BOLD)


if __name__ == '__main__':
    import os
    from pathlib import Path

    m = Navigation(graph_path=      Path(os.environ['WS_DIR']) / 'src/navigation/landmarks/reach.txt',
                   frame_id=        'map',
                   model_name=      os.environ['TURTLEBOT_MODEL'],
                   in_simulation=   bool(rospy.get_param('simulation',0)),
                   autorun=         True,
                   verbose=         False)
    try:
        m.start()
    except rospy.ROSInterruptException:
        pass
    
    
