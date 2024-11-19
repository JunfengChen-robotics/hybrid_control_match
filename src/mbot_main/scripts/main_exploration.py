import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped

import math

class SingleMission:
    def __init__(self, mission_type='', mission_pose=Pose(), target_color=None):
        self.type = mission_type
        self.pose = mission_pose  # geometry_msgs.msg/Pose
        if self.type == 'SECURE':
            self.color = target_color
        else:
            self.color = None

class MissionSet:
    def __init__(self, planner):
        self.mission_set = [] 
        self.planner = planner

    def prior(self):
        """返回当前优先级最高的任务"""
        # 方案一：按照和机器人当前的距离排序
        # trans, rot = self.planner.get_robot_position()
        # min_dis = math.inf
        # min_index = 0
        # for i in range(len(self.mission_set)):
        #     dis = math.sqrt((trans[0]-self.mission_set[i].pose.position.x)**2 + (trans[1]-self.mission_set[i].pose.position.y)**2)
        #     if dis < min_dis:
        #         min_dis = dis
        #         min_index = i
        # return self.mission_set[min_index]
    
        # 方案二：选第一个
        return self.mission_set[0]

    def add_secure_mission(self, mission_pose):
        """添加新的救援SECURE任务"""
        # 判断坐标是否合法

        # 通过坐标判断是否已经存在该任务

        # 添加任务
        self.mission_set.append(SingleMission('SECURE', mission_pose))
        return True

    def add_explore_mission(self, mission_poses):
        """添加新的探索EXPLORE任务"""
        # 清空之前的探索任务
        self.mission_set = [mission for mission in self.mission_set if mission.type != 'EXPLORE']

        # 添加新的探索任务
        for mission_pose in mission_poses.poses:
            self.mission_set.append(SingleMission('EXPLORE', mission_pose))
        return True
    
    def is_empty(self):
        """判断任务集是否为空"""
        if len(self.mission_set) == 0:
            return True
        else:
            return False

class MissionPlanner:
    def __init__(self):
        rospy.init_node('mission_planner')
        rospy.loginfo('Mission planner node initialized')

        self.mission_set = MissionSet(self)

        self.rate = rospy.Rate(10) 


        # ROS Interfaces
        rospy.Subscriber('object_detect_pose', Pose, self.object_detect_pose_callback)  # 订阅识别的物体的位置

        first_ = rospy.wait_for_message('frontiers', PoseArray)  # 等待边界探索的目标点
        self.mission_set.add_explore_mission(first_)
        rospy.Subscriber('frontiers', PoseArray, self.frontiers_callback)  # 订阅边界探索的目标点

        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)  # 发布目标点

        # Initialize tf listener
        self.tf_listener = tf.TransformListener()

        # 任务开始
        self.start()

    
    def object_detect_pose_callback(self, pose):
        rospy.loginfo('Object detected at pose: {}'.format(pose))
        self.mission_set.add_secure_mission(pose)
    
    def frontiers_callback(self, pose_arr):
        self.mission_set.add_explore_mission(pose_arr)
    
    def get_robot_position(self):
        """
        获取机器人在/map坐标系中的位置

        尝试获取/base_footprint在/map坐标系中的位置和方向。

        Returns:
            trans (tuple): 机器人在/map坐标系中的平移 (x, y, z)
            rot (tuple): 机器人在/map坐标系中的旋转 (x, y, z, w)
        """
        try:
            self.tf_listener.waitForTransform('/map', '/base_footprint', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("TF Exception")
            return None, None

    
    def start(self):
        rospy.loginfo('Starting mission planner')
        while not rospy.is_shutdown():
            # 若任务集为空，则任务结束
            if self.mission_set.is_empty():
                rospy.loginfo('No mission in mission set. Mission Complete.')
                break
            # 否则，继续执行任务
            else:
                ## 1. 检查边界点list，输出下一目标点
                mission_now = self.mission_set.prior()

                if mission_now.type == 'SECURE':
                    
                    ## 2. 调用运动规划器，发布目标点
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = rospy.Time.now()  
                    pose_stamped.pose = mission_now.pose
                    self.move_base_goal_pub.publish(pose_stamped)

                elif mission_now.type == 'EXPLORE':
                    ## 2. 调用运动规划器，发布目标点
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = mission_now.pose


                    self.move_base_goal_pub.publish(pose_stamped)

                
                else:
                    rospy.loginfo('Unknown mission type. Skip this mission.')

            self.rate.sleep()



if __name__ == '__main__':
    mp = MissionPlanner()
