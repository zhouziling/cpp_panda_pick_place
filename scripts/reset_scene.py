#!/usr/bin/env python

from panda_pick_and_place.srv import ResetScene
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose
import rospy
import rospkg

def reset_cube(model_name,pos_x,pos_y):

    del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) 
    del_model_prox(str(model_name))

    # get model path
    model_path = rospkg.RosPack().get_path('panda_pick_and_place')+"/models/"

    # load model URDF
    model_xml = ''
    with open (model_path + str(model_name) + ".urdf", "r") as block_file:
        model_xml=block_file.read().replace('\n', '')

    # spawn model
    model_pose = Pose()
    model_pose.position.x = pos_x
    model_pose.position.y = pos_y
    model_pose.position.z = 0.1

    add_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    add_model( str(model_name), model_xml, "model_namespace", model_pose, "world")

def handle_reset_scene(request):
    try:
        reset_cube("red_cube",0.6, 0)
        reset_cube("blue_cube", 0.04, -0.4)
        reset_cube("green_cube", -0.3, 0.4)

        status = "scene reset DONE"

    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed: %s", e)
        status = "scene reset FAILED"

    return status

def reset_scene_server():
    rospy.init_node('reset_scene')
    s = rospy.Service('reset_scene_service', ResetScene, handle_reset_scene)


if __name__=="__main__":
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    
    reset_scene_server()
    
    rospy.spin()