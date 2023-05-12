from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler # reference: http://docs.ros.org/en/jade/api/tf/html/python/transformations.html

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
            self.quaternion.w = 1
        
        def __repr__(self):
            return self.__str__()
        
        def __str__(self) -> str:
            euler = f'({math.degrees(self.roll):.4f}, {math.degrees(self.pitch):.4f}, {math.degrees(self.yaw):.4f})°'
            quat = f'({self.quaternion.x:.4f}, {self.quaternion.y:.4f}, {self.quaternion.z:.4f}, {self.quaternion.w:.4f})'
            data = '{' + f"\n\t(roll, pitch, yaw): {euler}," + f"\n\tquaternion: {quat}\n" + '}'
            return data
      
    def __init__(self, id='point', position=None, orientation=None, degrees2radians=True):
        """Supporting class. It can be seen as an extension of the ripo Pose of ros.

        Args:
            id (str, optional): node identification name. Defaults to 'point'.
            position (optional): position (x,y,z) can be passed as a list or Point (ros type). Defaults to None, means (0,0,0) position.
            orientation (optional): orientation can be passed as a list of Euler angles (roll, pitch, yaw), as a list of elements of a quaternion, or as a Quaternion (type ros). The default value is None, that is, (0,0,0) in Euler angles or (0,0,0) in quaterion.
            degrees2radians (bool, optional): if True converts the given Euler angles from degrees to radians. Defaults to True.
        """
        self.id = id
        
        self.position = Point()
        if position is not None:
            if isinstance(position, Point):
                self.position = position
            else:
                self.position = Point(position[0], position[1], position[2])
        
        self.set_orientation(orientation,degrees2radians)
                
    def set_orientation(self, orientation, degrees2radians=True):
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
                # quaternion as a list
                self.orientation.quaternion = Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
                
                roll, pitch, yaw = euler_from_quaternion(orientation)
                self.orientation.roll = roll
                self.orientation.pitch = pitch
                self.orientation.yaw = yaw
            else:
                # list of euler angles in degrees
                if degrees2radians:
                    orientation = list(map(math.radians,orientation))
                self.orientation.roll = orientation[0]
                self.orientation.pitch = orientation[1]
                self.orientation.yaw = orientation[2]
                tmp = quaternion_from_euler(orientation[0],orientation[1],orientation[2])
                self.orientation.quaternion = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
            
    def get_position_numpy(self):
        return np.array([self.position.x,self.position.y,self.position.z])
    
    def get_euler_numpy(self):
        return np.array([self.orientation.roll,self.orientation.pitch,self.orientation.yaw])
    
    def get_quaternion_numpy(self):
        return np.array(self.orientation.quaternion.x,self.orientation.quaternion.y,self.orientation.quaternion.z,self.orientation.quaternion.w)
    
    @staticmethod
    def from_pose_to_ppoint(pose:Pose):
        # quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return PPoint(position=pose.position, orientation=pose.orientation)
    
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

