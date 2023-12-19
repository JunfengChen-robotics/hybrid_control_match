#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import os
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
import argparse

goal_model_1_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_1', 'model.sdf')
goal_model_2_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_2', 'model.sdf')
goal_model_3_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_3', 'model.sdf')
goal_model_4_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_4', 'model.sdf')
goal_model_5_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_5', 'model.sdf')
goal_model_6_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_6', 'model.sdf')
goal_model_7_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_7', 'model.sdf')
goal_model_8_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_8', 'model.sdf')
goal_model_9_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..', 'mbot_recognition', 'worlds', 'Target_9', 'model.sdf')
goal_model_10_dir = os.path.join(os.path.split(os.path.realpath(__file__))[0], '..', '..','mbot_recognition', 'worlds', 'Target_10', 'model.sdf')
goal_model_list = [f"goal_model_{i}_dir" for i in range(1,11,1)]

class Env():
  def __init__(self, N, obj_pos_list):
    
    self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
    self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
    self.goal_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    self.N_obj = N
    self.obj_pos_list = obj_pos_list
    time.sleep(1.0)
    
    
  def build_env(self):
    
    print("=============================> Build Environment <=============================")
    rospy.wait_for_service('/gazebo/pause_physics')
    try:
        self.pause_proxy()
    except (rospy.ServiceException) as e:
        print("gazebo/pause_physics service call failed")
    
    time.sleep(0.1)
    
    rospy.wait_for_service('/gazebo/delete_model')
    
    try:
        for i in range(10):
            self.del_model(f"target_{i}")
            if i == self.N_obj-1:
               break
    except (rospy.ServiceException) as e:
        print("gazebo/delete_model service call failed")
        
    time.sleep(0.1)

    #BUILD THE TARGETS
    
  
    for i in range(self.N_obj):
      rospy.wait_for_service('/gazebo/spawn_sdf_model')
      try:
          goal_urdf = open(goal_model_list[i], "r").read()
          target = SpawnModel
          target.model_name = f'target_{i+1}'  # the same with sdf name
          target.model_xml = goal_urdf
          goal_model_position = Pose()
          temp_x = self.obj_pos_list[i][0]
          temp_y = self.obj_pos_list[i][1]
          goal_model_position.position.x, goal_model_position.position.y = temp_x, temp_y
          self.goal_model(target.model_name, target.model_xml, 'namespace', goal_model_position, 'world')
      except (rospy.ServiceException) as e:
          print(f"/gazebo/failed to build the target_{i+1}")
        
      time.sleep(0.5)
      
      
      
if __name__ == "__main__":
    
    rospy.init_node("gen_obj", anonymous=True)
    parser = argparse.ArgumentParser(description='Set Object Number')
    parser.add_argument('--num', type=int, default=2,
                        help='set the number of objects')
    args = parser.parse_args()
    
    obj_pos_list = [
                    [0,0],
                    [2,2],
                    [4,4]
                  ]
    
    env = Env(args.num)
    
    env.build_env()
    
    
    
    
    
        
        
      
    
    
    
