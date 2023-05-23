import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist, Pose, Pose2D, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty

import graph
from typing import Tuple
import math
import json
import numpy as np

def str2tuple(tuple_str):
    return tuple(map(float, tuple_str.replace('(','').replace(')','').split(',')))

def str2list(list_str):
    return list(map(float, list_str.replace('[','').replace(']','').split(',')))

def read_graph(filename):    
    infile = open(filename, 'r')
    V=set()
    E=dict()
    
    for line in infile:
        nodes=line.split()
        if nodes[0] not in V:
            node0 = str2tuple(nodes[0])
            V.add(node0)
        if nodes[1] not in V:
            node1 = str2tuple(nodes[1])
            V.add(node1)
        E[(node0,node1)]=int(nodes[2])
    infile.close()
    return V, E

def build_graph(V, E, is_directed = False, vertex_map: dict = None) -> Tuple[graph.Graph, dict]:
    G = graph.Graph(directed=is_directed)
    
    """
    NOTE: La funzione insert_edge() vuole esattamente un riferimento ad un vertice del grafo
    Quindi, se passassi un nuovo oggetto Vertex, anche se con lo stesso valore di quello che è presente nel grafico, non lo accetterebbe perché non ne troverebbe l'istanza nel grafo
    Il dizionario vertexMap, serve proprio a salvare il riferimento tra l'elemento e il vertice. L'univocità dell'elemento mi è garantita dal vatto che è contenuto nel set V
    """    
    vertex_map = vertex_map if vertex_map is not None else dict()
    for vertex in V:
        vertex_map[vertex] = G.insert_vertex(vertex)

    for edge in E:
        origin = vertex_map[edge[0]]
        destination = vertex_map[edge[1]]
        element = E[edge]
        G.insert_edge(origin, destination, element)

    return G, vertex_map

def get_vertices_numpy(vertex_map:dict):
    """Prende vertex_map e restituisce i vertici come matrice numpy

    Args:
        vertex_map (dict): dizionario dei vertici
    """
    v = np.array(list(vertex_map.keys()))
    return v

def to_pose2D(pose=None, position=[0,0], orientation=0) -> Pose2D:
    if isinstance(pose,Pose):
        yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z,pose.orientation.w])[2]
        return Pose2D(pose.position.x, pose.position.y, yaw)
    elif isinstance(pose,PoseWithCovarianceStamped):
        tmp = pose.pose.pose
        yaw = euler_from_quaternion([tmp.orientation.x, tmp.orientation.y, tmp.orientation.z, tmp.orientation.w])[2]
        return Pose2D(tmp.position.x, tmp.position.y, yaw)
    else:
        if isinstance(orientation,list) or isinstance(orientation,tuple):
            if len(orientation) > 3:
                yaw = euler_from_quaternion(orientation)[2]
            elif len(orientation) > 2:
                yaw = orientation[2]
        else:
            yaw = orientation
        return Pose2D(position[0], position[1], yaw)
    
def to_pose(pose=None, position=[0,0,0], orientation=[0,0,0,1]) -> Pose:
    if isinstance(pose,Pose2D):
        point = Point(pose.x,pose.y,0)
        tmp = quaternion_from_euler(0,0,pose.theta)
        q = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        return Pose(position=point,orientation=q)
    else:
        if isinstance(position,list) and len(position) < 3:
            position.extend([0])
        point = Point(position[0],position[1],position[2])
        if isinstance(orientation,list) or isinstance(orientation,tuple):
            if len(orientation) > 3:
                q = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
            elif len(orientation) > 2:
                tmp = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
                q = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        else:
            tmp = quaternion_from_euler(0,0,orientation)
            q = Quaternion(tmp[0], tmp[1], tmp[2], tmp[3])
        return Pose(position=point,orientation=q)

def get_position_numpy(pose,to3D=False) -> np.ndarray:
    if isinstance(pose,Pose):
        pose : Pose        
        return np.array([pose.position.x, pose.position.y, pose.position.z])
    elif isinstance(pose,Pose2D):
        pose : Pose2D
        if to3D:
            return np.array([pose.x,pose.y,0])
        else:
            return np.array([pose.x,pose.y])
    elif isinstance(pose,PoseWithCovarianceStamped):
        return np.array([pose.pose.pose.position.x, pose.pose.pose.position.y,pose.pose.pose.position.z])
    elif isinstance(pose,list):
        return np.array(pose)
    elif isinstance(pose,np.ndarray):
        return pose
    else:
        return None

def jump_to(model_name, point:Pose, hard_reset=False, verbose=False):       
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position = point.position
    state_msg.pose.orientation = point.orientation
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

class NpEncoder(json.JSONEncoder):
                def default(self, obj):
                    if isinstance(obj, np.integer):
                        return int(obj)
                    if isinstance(obj, np.floating):
                        return float(obj)
                    if isinstance(obj, np.ndarray):
                        return obj.tolist()
                    return json.JSONEncoder.default(self, obj)
