import rospy
import tf
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import math

class Task:
    def __init__(self, task_type='', task_pose=Pose(), target_color=None):
        self.type = task_type
        self.pose = task_pose  # geometry_msgs.msg/Pose
        if self.type == 'SECURE':
            self.color = target_color
        else:
            self.color = None

class TaskSet:
    def __init__(self, planner):
        self.task_set = [] 
        self.planner = planner
        self.secure_set = []

    def prior(self):
        """返回当前优先级最高的任务"""
        # 方案一：按照和机器人当前的距离排序
        # trans, rot = self.planner.get_robot_position()
        # min_dis = math.inf
        # min_index = 0
        # for i in range(len(self.task_set)):
        #     dis = math.sqrt((trans[0]-self.task_set[i].pose.position.x)**2 + (trans[1]-self.task_set[i].pose.position.y)**2)
        #     if dis < min_dis:
        #         min_dis = dis
        #         min_index = i
        # return self.task_set[min_index]
    
        # 方案二：选第一个
        return self.task_set[0]

    def add_secure_task(self, task_point):
        """添加新的救援SECURE任务"""
        task_pose = Pose()
        task_pose.position = task_point
        task_pose.orientation.w = 1
        # 判断坐标是否合法
        if task_pose.position.x < -3 or task_pose.position.x > 3 or task_pose.position.y < -2.5 or task_pose.position.y > 3.5:
            rospy.logwarn('Exceed the map boundary. Task not added.')
            return False

        # 通过坐标判断是否已经存在该任务
        for pose in self.secure_set:
            if (pose.position.x - task_pose.position.x)**2 + (pose.position.y - task_pose.position.y)**2 < 0.5**2:
                return False

        # 添加任务
        # self.task_set.append(Task('SECURE', task_pose))  #TODO: 考虑如何删除已经完成的任务
        self.secure_set.append(task_pose)

        # 发布Marker
        marker_array = MarkerArray()
        for i, pose in enumerate(self.secure_set):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'secure_marker'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.planner.secure_marker_pub.publish(marker_array)
        return True

    def add_explore_task(self, task_poses):
        """添加新的探索EXPLORE任务"""
        # 清空之前的探索任务
        self.task_set = [task for task in self.task_set if task.type != 'EXPLORE']

        # 添加新的探索任务
        for task_pose in task_poses.poses:
            self.task_set.append(Task('EXPLORE', task_pose))
        return True
    
    def is_empty(self):
        """判断任务集是否为空"""
        if len(self.task_set) == 0:
            return True
        else:
            return False

class taskPlanner:#
    def __init__(self):
        rospy.init_node('task_planner')
        rospy.loginfo('task planner node initialized')

        self.task_set = TaskSet(self)

        self.rate = rospy.Rate(3) 


        # ROS Interfaces
        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)  # 发布目标点

        self.secure_marker_pub = rospy.Publisher('secure_marker', MarkerArray, queue_size=10)  # 发布救援目标点

        # Initialize tf listener
        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('object_detect_pose', PointStamped, self.object_detect_pose_callback)  # 订阅识别的物体的位置

        first_ = rospy.wait_for_message('frontiers', PoseArray)  # 等待边界探索的目标点
        self.task_set.add_explore_task(first_)
        rospy.Subscriber('frontiers', PoseArray, self.frontiers_callback)  # 订阅边界探索的目标点

        
        # 任务开始
        self.start()

    
    def object_detect_pose_callback(self, msg):
        self.task_set.add_secure_task(msg.point)
    
    def frontiers_callback(self, pose_arr):
        self.task_set.add_explore_task(pose_arr)
    
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
        rospy.loginfo('Starting task planner')
        while not rospy.is_shutdown():
            # 若任务集为空，则任务结束
            if self.task_set.is_empty():
                rospy.loginfo('No task in task set. task Complete.')
                break
            # 否则，继续执行任务
            else:
                ## 1. 检查边界点list，输出下一目标点
                task_now = self.task_set.prior()

                if task_now.type == 'SECURE':
                    
                    ## 2. 调用运动规划器，发布目标点
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = rospy.Time.now()  
                    pose_stamped.pose = task_now.pose
                    self.move_base_goal_pub.publish(pose_stamped)

                elif task_now.type == 'EXPLORE':
                    ## 2. 调用运动规划器，发布目标点
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose = task_now.pose


                    self.move_base_goal_pub.publish(pose_stamped)

                
                else:
                    rospy.loginfo('Unknown task type. Skip this task.')

            self.rate.sleep()



if __name__ == '__main__':
    tp = taskPlanner()
