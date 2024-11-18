import rospy
from geometry_msgs.msg import Pose

class SingleMission:
    def __init__(self, mission_type='', mission_pose=Pose(), target_color=None):
        self.type = mission_type
        self.pose = mission_pose  # geometry_msgs.msg/Pose
        if self.type == 'SECURE':
            self.color = target_color
        else:
            self.color = None

class MissionSet:
    def __init__(self):
        self.mission_set = [] 

    def prior(self):
        """返回当前优先级最高的任务"""
        return None

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
        for mission_pose in mission_poses:
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

        self.mission_set = MissionSet()

        self.rate = rospy.Rate(10) 


        # ROS Interfaces
        rospy.Subscriber('object_detect_pose', Pose, self.object_detect_pose_callback)  # 订阅识别的物体的位置

        rospy.Subscriber('frontiers', Pose, self.frontiers_callback)  # 订阅边界探索的目标点

        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', Pose, queue_size=10)  # 发布目标点

        # 任务开始
        self.start()

    
    def object_detect_pose_callback(self, pose):
        rospy.loginfo('Object detected at pose: {}'.format(pose))
        self.mission_set.add_secure_mission(pose)
    
    def frontiers_callback(self, poses):
        # TODO: poses is a list of Pose
        rospy.loginfo('Frontier detected at poses: {}'.format(poses))
        self.mission_set.add_explore_mission(poses)

    
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

                ## 2. 调用运动规划器，发布目标点
                self.move_base_goal_pub.publish(mission_now.pose)

            self.rate.sleep()



if __name__ == '__main__':
    mp = MissionPlanner()
