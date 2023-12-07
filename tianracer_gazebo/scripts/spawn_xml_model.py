#! /usr/bin/python3
# -*- coding: utf-8 -*-

'''
Description: spawn sdf or urdf model in Gazebo by read spawn_pose.yaml
version: v0.1
Author: sujit-168 
Date: 2023-10-22 22:14:25

Copyright (c) 2023 by Tianbot , All Rights Reserved. 
'''

import rospy
import rospkg as pkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Pose, Quaternion
import waypoint_race.utils as utils
import yaml

class SpawnXmlModel:
    
    def __init__(self, model_path, pose_path, model_type):  
        '''
        model_path : sdf or urdf path absolutely
        pose_path : a .yaml file contain some pose
        model_type : to select /gazebo/spawn_sdf_model or /gazebo/spawn_urdf_model
        '''
        
        self.pose = Pose()
        self._counter = 0
        rospy.init_node('spawn_model', anonymous=True)
        self.spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_' + model_type + '_model', SpawnModel)
        self.model_xml = self.read_xml(model_path)
        self._pose_data = utils.get_waypoints(pose_path)
        rospy.loginfo("Reached the end of the waypoint list: %d", len(self._pose_data))

    def read_xml(self, file_path):
        '''
        read sdf or urdf model from file
        '''
        try:
            f = open(file_path,'r')
            sdf = f.read()
            f.close()
        except:
            rospy.logerr("Could not open file: %s", file_path)
            sdf = None
        return sdf

    def update_pose(self):
        '''
        update self.pose value by add self._counter
        '''
        next_pose = None
        next_pose = self._pose_data[self._counter]
        self.pose = utils.create_geometry_pose(next_pose)
        self._counter = self._counter + 1
        rospy.loginfo("Received %d goal \n pose: %s", self._counter, self.pose)


    def check_session(self):
        '''
        check if the pose in yaml has updated completely
        '''
        if self._counter == len(self._pose_data):
            Finished = True
        else:
            self.update_pose()
            Finished = False
        return Finished
  
    def spawn_model(self, model_name):
        '''
        model_name: mode_name spawned in gazebo 
        '''
        spawn_request = SpawnModelRequest()
        resp = SpawnModelResponse()
        
        spawn_request.model_name = model_name + '_'  + str(self._counter)

        rospy.loginfo("Spawning model: %s", spawn_request.model_name)

        spawn_request.model_xml = self.model_xml
        spawn_request.initial_pose = self.pose
        spawn_request.robot_namespace = ""
        spawn_request.reference_frame = "world"

        rospy.wait_for_service('/gazebo/spawn_' + model_type + '_model')
        resp = self.spawn_model_service(spawn_request)

        return resp

if __name__ == '__main__':

    # set some default params
    pkg = pkg.RosPack()
    pose_data_path = pkg.get_path('tianracer_gazebo') + '/config/spawn.yaml'
    model_path = pkg.get_path('tianracer_gazebo') + '/model/construction_cone/model.sdf'

    # get params from parameter server
    model_name = rospy.get_param("model_name", "model_name")
    model_path = rospy.get_param("model_path", model_path)
    model_type = rospy.get_param("model_type", "sdf")
    pose_path = rospy.get_param("pose_data_path", pose_data_path)

    spawner = SpawnXmlModel(model_path, pose_path, model_type="sdf")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        Finished = spawner.check_session()
        if Finished:
            break
            rospy.loginfo("Spawn session has finished!")
        else:
            spawn_resp = spawner.spawn_model(model_name)
            rospy.loginfo('Model spawn {} : {},\n{}'.format(spawner._counter, spawn_resp.success, spawn_resp.status_message))

