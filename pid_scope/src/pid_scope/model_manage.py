import os
import rospy
from gazebo_msgs.srv import DeleteModel

def spawn_model(package_name: str, launch_file: str):
    os.system(f"roslaunch {package_name} {launch_file}")

def delete_model(name: str):
    rospy.ServiceProxy("gazebo/delete_model", DeleteModel).call(name)