import roslaunch
import rospy
import rospkg
from pathlib import Path
from utils import get_turtle_ip
import argparse
import subprocess

def sftp(turtle_ip):
    '''
    This code is loading the `realsense.py` file on a remote robot using the SFTP protocol. It first
    creates two path objects using the `realsense_pkg_path` variable and the relative paths to the
    `realsense.py` file and the `sftp.bash` script. Then it checks if both files exist using the
    `exists()` method. If they exist, it creates a command string with the `turtle_ip` variable and
    the paths to the `realsense.py` file and the `sftp.bash` script. Finally, it runs the command
    using the `subprocess` module.
    '''
    realsense_src_path = realsense_pkg_path.joinpath("src/realsense.py")
    sftp_path = realsense_pkg_path.joinpath("src/camera_launch/sftp.bash")
    assert realsense_src_path.exists()
    assert sftp_path.exists()
    cmd = f"bash {str(sftp_path)} {turtle_ip} {str(realsense_src_path)}"
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    p.check_returncode()
    
def del_src_file(turtle_ip):
    '''
    This code is deleting the `realsense.py` file from the turtlebot's workspace using SSH protocol.
    It first creates a command string with the `turtle_ip` variable and the path to the
    `realsense.py` file on the turtlebot. Then it runs the command using the `subprocess` module and
    checks the return code to ensure that the command was executed successfully.
    ''' 
    cmd = f"sshpass -p turtlebot ssh -o StrictHostKeyChecking=no ubuntu@{turtle_ip} 'rm /home/ubuntu/realsense/catkin_ws/src/realsense/src/realsense.py'"
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    p.check_returncode()

if __name__ == "__main__":
    parser = argparse.ArgumentParser() # Argument paser
    parser.add_argument("--query", required=False, dest="query", default=None, action="store_true") 
    args = parser.parse_args()
    rospkg_obj = rospkg.RosPack()
    
    realsense_pkg_path = Path(rospkg_obj.get_path("realsense"))
    turtle_ip = get_turtle_ip() # Gets the turtlebot IP reading the history.db file
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) #Setup roslaunch
    roslaunch.configure_logging(uuid)
    
    if args.query is not None:
        launch_file_path = realsense_pkg_path.joinpath("launch/query_camera.launch")
    else:
        sftp(turtle_ip)
        launch_file_path = realsense_pkg_path.joinpath("launch/realsense.launch")
    cli_args = [str(launch_file_path), f"ip:={turtle_ip}"] # Parsing roslaunch arguments
    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    parent.start() # Starting nodes
    parent.spin()
    parent.shutdown()
    if args.query is None:
        del_src_file(turtle_ip)