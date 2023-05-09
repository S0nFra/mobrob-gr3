from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math
import numpy as np

""" Module with auxiliary functions """

class PPoint():
    
    class Orientation():
        __slots__ = ['roll', 'pitch', 'yaw', 'quaternion']
        
        def __init__(self):
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
            self.quaternion = Quaternion()
        
        def __repr__(self):
            return self.__str__()
        
        def __str__(self) -> str:
            euler = f'({math.degrees(self.roll):.4f}, {math.degrees(self.pitch):.4f}, {math.degrees(self.yaw):.4f})°'
            quat = f'({self.quaternion.x:.4f}, {self.quaternion.y:.4f}, {self.quaternion.z:.4f}, {self.quaternion.w:.4f})'
            data = '{' + f"\n\t(roll, pitch, yaw): {euler}," + f"\n\tquaternion: {quat}\n" + '}'
            return data
      
    def __init__(self, id='point', position=None, orientation=None):
        self.id = id
        
        self.position = Point()
        if position is not None:
            if isinstance(position, Point):
                self.position = position
            else:
                self.position = Point(position[0], position[1], position[2])
        
        # da quaternione a angoli di eulero (nota: gli angoli sono espressi in radianti)
        # reference: http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        self.orientation = self.Orientation()
        if orientation is not None:
            if isinstance(orientation, Quaternion):
                self.orientation.quaternion = orientation
                orientation = [orientation.x, orientation.y, orientation.z, orientation.w]

                roll, pitch, yaw = euler_from_quaternion(orientation)
                self.orientation.roll = roll
                self.orientation.pitch = pitch
                self.orientation.yaw = yaw
            elif len(orientation) > 3:
                self.orientation.quaternion = Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
                
                roll, pitch, yaw = euler_from_quaternion(orientation)
                self.orientation.roll = roll
                self.orientation.pitch = pitch
                self.orientation.yaw = yaw
            else:    
                self.orientation.roll = math.radians(orientation[0])
                self.orientation.pitch = math.radians(orientation[1])
                self.orientation.yaw = math.radians(orientation[2])
                orientation = list(map(math.radians,orientation))
                tmp = quaternion_from_euler(orientation[0],orientation[1],orientation[2])
                self.orientation.quaternion = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
            
    def get_position_numpy(self):
        return np.array([self.position.x,self.position.y,self.position.z])
    
    @staticmethod
    def from_pose_to_ppoint(pose:Pose):
        quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return PPoint(position=pose.position, orientation=quat)
    
    @staticmethod
    def from_ppoint_to_pose(ppoint)->Pose:
        return Pose(position=ppoint.position, orientation=ppoint.orientation.quaternion)
    
    def to_pose(self)->Pose:
        return Pose(position=self.position, orientation=self.orientation.quaternion)
        
    def __repr__(self) -> str:
        return self.__str__()
        
    def __str__(self) -> str:
        pos = f"({self.position.x:.4f}, {self.position.y:.4f}, {self.position.z:.4f})"
        data = '{ ' + f'id: {self.id}\n' + \
                f'  position: {pos}\n' + \
                f'  orientation: {self.orientation}' + '}'
        return data

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def jump_to(model_name, point: PPoint, hard_reset=False, verbose=False):        
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = point.position.x
    state_msg.pose.position.y = point.position.y
    state_msg.pose.position.z = point.position.z
    state_msg.pose.orientation = point.orientation.quaternion
    state_msg.twist = Twist()
    
    try:
               
        if hard_reset:
            rospy.wait_for_service('/gazebo/reset_simulation')
            rospy.wait_for_service('/gazebo/reset_world')
        
            rospy.ServiceProxy('/gazebo/reset_simulation', Empty)()
            rospy.ServiceProxy('/gazebo/reset_world', Empty)()

        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_state( state_msg )

        if verbose:
            print("### INIT POSE ###")
            print(point,'\n')

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def no_op(*args, **kwargs):
    pass


