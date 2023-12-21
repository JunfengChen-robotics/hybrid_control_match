#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import os
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
import time
from std_srvs.srv import Empty
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, PointStamped
import argparse

goal_model_1_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_1',
                                'model.sdf')
goal_model_2_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_2',
                                'model.sdf')
goal_model_3_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_3',
                                'model.sdf')
goal_model_4_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_4',
                                'model.sdf')
goal_model_5_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_5',
                                'model.sdf')
goal_model_6_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_6',
                                'model.sdf')
goal_model_7_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_7',
                                'model.sdf')
goal_model_8_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_8',
                                'model.sdf')
goal_model_9_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_9',
                                'model.sdf')
goal_model_10_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'Target_10',
                                 'model.sdf')
goal_model_list = [goal_model_1_dir, goal_model_2_dir, goal_model_3_dir, goal_model_4_dir, goal_model_5_dir,
                   goal_model_6_dir, goal_model_7_dir, goal_model_8_dir, goal_model_9_dir, goal_model_10_dir]

position = []


def callback(msg):
    x, y = msg.point.x, msg.point.y
    global position
    position.clear()
    position.append(x)
    position.append(y)
    rospy.loginfo("The position of this robot is " + str(position))


class Env():
    def __init__(self, N, obj_pos_list):

        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.goal_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.del_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.N_obj = N
        self.obj_pos_list = obj_pos_list
        rospy.Subscriber('robot_position', PointStamped, callback)
        self.flag_publisher = rospy.Publisher('flag', Int32, queue_size=10)
        # self.delete_num = 0
        self.delete_list = []

        time.sleep(1.0)

    def build_env(self):

        print("=============================> Build Environment <=============================")
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/pause_physics service call failed")

        time.sleep(0.1)

        # delete existing targets
        rospy.wait_for_service('/gazebo/delete_model')

        # try:
        #     for i in range(10):
        #         self.del_model(f"target_{i + 1}")
        #         if i == self.N_obj - 1:
        #             break
        # except (rospy.ServiceException) as e:
        #     print("gazebo/delete_model service call failed")
        #
        # time.sleep(0.1)

        # BUILD THE TARGETS
        for i in range(self.N_obj):
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                goal_str = goal_model_list[i]
                goal_urdf = open(goal_str, "r").read()
                target = SpawnModel
                target.model_name = f'target_{i + 1}'  # the same with sdf name
                target.model_xml = goal_urdf
                goal_model_position = Pose()
                temp_x = self.obj_pos_list[i][0]
                temp_y = self.obj_pos_list[i][1]
                goal_model_position.position.x, goal_model_position.position.y = temp_x, temp_y
                self.goal_model(target.model_name, target.model_xml, 'namespace', goal_model_position, 'world')
            except (rospy.ServiceException) as e:
                print(f"/gazebo/failed to build the target_{i + 1}")

            time.sleep(0.5)

        self.unpause_proxy()

    def Distance(self, point):
        L = 0.4 ** 2
        for i in range(self.N_obj):
            if i in self.delete_list:
                pass
            elif (self.obj_pos_list[i][0] - point[0]) ** 2 + (self.obj_pos_list[i][0] - point[0]) ** 2 < L:
                return i
        return -1

    def dynamically_delete(self):
        while not rospy.is_shutdown():
            global position
            if len(position) == 0:
                pass
            else:
                delete_index = self.Distance(position)
                if delete_index == -1:
                    pass
                else:
                    self.pause_proxy()
                    self.del_model(f"target_{delete_index + 1}")
                    # del self.obj_pos_list[delete_index]
                    # self.N_obj = self.N_obj - 1
                    # self.delete_num = self.delete_num + 1
                    self.delete_list.append(delete_index)
                    rospy.loginfo("delete the rescue goal: " + str(delete_index))
                    self.unpause_proxy()
                    flag = 0
                    self.flag_publisher.publish(flag)

            time.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("gen_obj_gazebo", anonymous=True)

    obj_pos_list = [
        [0, 0],
        [2, 0],
        [1.5, -1],
        [-0.5, -1],
        [-1.5, 2],
        [-2.5, 1]
    ]

    env = Env(6, obj_pos_list)

    env.build_env()

    env.dynamically_delete()