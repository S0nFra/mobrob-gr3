import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def jump_to(model_name, point, hard_reset=False, verbose=False):
    
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position = point.position
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