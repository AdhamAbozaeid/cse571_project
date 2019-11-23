#! /usr/bin/python

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
import os, sys

def main():
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
    rospy.init_node('insert_object',log_level=rospy.INFO)
    f_read = open(os.path.expanduser("~") + "/.gazebo/models/beer/model2.sdf",'r')
    sdff = f_read.read()
    f_read.close()

    for i in range(1):

        #rospy.wait_for_service('gazebo/delete_model')
        #delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', SpawnModel)
        #delete_model_prox("fuelstation{}{}".format(initial_pose.position.x, initial_pose.position.y))
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        print("got it")
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        print(spawn_model_prox)
        initial_pose = Pose()
        initial_pose.position.x = 0.5
        initial_pose.position.y = 0.5
        initial_pose.position.z = 0
        print(initial_pose)
        spawn_model_prox("fuelstation{}{}".format(initial_pose.position.x+1000, initial_pose.position.y+10000), sdff, "robotos_name_space", initial_pose, "world")
        print(spawn_model_prox)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
