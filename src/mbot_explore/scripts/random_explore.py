import rospy
import random
import sys
import select
import tty
import termios
from geometry_msgs.msg import Pose, PoseArray
import tf
from tf.transformations import quaternion_from_euler
import math

class RandomExplore:
    def __init__(self):
        rospy.init_node('random_explore')
        rospy.loginfo('Random explore node initialized')

        self.frontier_pub = rospy.Publisher('/frontiers', PoseArray, queue_size=1)

        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        rospy.Timer(rospy.Duration(0.1), self.keyboard_check_timer_callback)

        # rospy.Timer(rospy.Duration(10), self.timer_callback)

        rospy.spin()

    def keyboard_check_timer_callback(self, event):
        """检查键盘输入并发布探索的目标点"""
        if self.is_key_pressed():
            key = sys.stdin.read(1)
            if key == ' ':
                self.publish_random_frontier()

    def is_key_pressed(self):
        """检查是否有键盘输入"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def publish_random_frontier(self):
        """发布一组随机的探索目标点"""
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        for i in range(3):
            pose = Pose()
            pose.position.x = random.uniform(-3, 3)
            pose.position.y = random.uniform(-2.5, 3.5)

            # 生成随机朝向
            yaw = random.uniform(-math.pi, math.pi)
            quaternion = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            pose_array.poses.append(pose)
        self.frontier_pub.publish(pose_array)
        rospy.loginfo('Publishing a new set of frontier points')

    def __del__(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)
    
    def timer_callback(self, event):
        self.publish_random_frontier()


if __name__ == '__main__':
    try:
        RandomExplore()
    except rospy.ROSInterruptException:
        pass