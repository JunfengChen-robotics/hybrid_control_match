#include<math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#define STATUS_EXPLORING    (0)
#define STATUS_CLOSE_TARGET (1)
#define STATUS_GO_HOME      (2)

#define GET_TARGET_SIZE     (90000)

ros::Publisher vel_pub;
ros::Publisher cmd_pub;
ros::Publisher voice_pub;

int status_flag = STATUS_EXPLORING;

// 接收到订阅的消息后，会进入消息回调函数
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Target pose: x:%0.6f, y:%0.6f, z:%0.6f", msg->position.x, msg->position.y, msg->position.z);

	if(status_flag==STATUS_EXPLORING)
	{
		status_flag=STATUS_CLOSE_TARGET;
		std_msgs::Int8 cmd;
		cmd.data=STATUS_CLOSE_TARGET;
		cmd_pub.publish(cmd);
	}
	else if(status_flag==STATUS_CLOSE_TARGET && msg->position.z > GET_TARGET_SIZE)
	{
		status_flag=STATUS_GO_HOME;
		std_msgs::Int8 cmd;
		cmd.data=STATUS_GO_HOME;
		cmd_pub.publish(cmd);

	}
	else if(status_flag==STATUS_CLOSE_TARGET)	//通过目标位置修改运动速度
	{
		//初始化Twist类型的消息
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = (150000 - msg->position.z) / 100000 * 0.3;
		vel_msg.angular.z = (640 - msg->posetion.x) / 640 * 0.3;
		//若目标中心点较图片x轴中心640偏右，那么按照计算公式angular.z是负值，负值机器人顺时针旋转即右转，使得中心点挪向中心；反之亦然

		//发布消息
		vel_pub.publish(vel_msg);
		ROS_INFO("Publish velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);
	}
}

int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "move_to_target");

	// 创建节点句柄
	ros::NodeHandle n;
	// 一个节点可以根据需要设定若干个订阅或者发布者
    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub = n.subscribe("/object_detect_pose", 10, poseCallback);
	// 创建一个Publisher，发布名为cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// 创建一个Publisher，发布名为cmd_vel的topic，消息类型为std_msgs::Int8，队列长度10
	cmd_pub = n.advertise<std_msgs::Int8>("/exploring_cmd", 10);

    // 循环等待回调函数
    ros::spin();

	return 0;
}