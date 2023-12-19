#include<math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define STATUS_EXPLORING    (0)
#define STATUS_CLOSE_TARGET (1)
#define STATUS_GO_HOME      (2)

#define GET_TARGET_SIZE     (90000)

ros::Publisher vel_pub;
ros::Publisher cmd_pub;

int status_flag = STATUS_EXPLORING;

// 接收到订阅的消息后，会进入消息回调函数
void poseCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Target pose: x:%0.6f, y:%0.6f, z:%0.6f", msg->point.x, msg->point.y, msg->point.z);

	if(status_flag==STATUS_EXPLORING)
	{
		status_flag=STATUS_CLOSE_TARGET;
		std_msgs::Int8 cmd;
		cmd.data=STATUS_CLOSE_TARGET;
		cmd_pub.publish(cmd);
	}
	// else if(status_flag==STATUS_CLOSE_TARGET && msg->position.z > GET_TARGET_SIZE)
	// {
	// 	// status_flag=STATUS_GO_HOME;
	// 	// std_msgs::Int8 cmd;
	// 	// cmd.data=STATUS_GO_HOME;
	// 	// cmd_pub.publish(cmd);

	// 	status_flag=STATUS_EXPLORING;
	// 	std_msgs::Int8 cmd;
	// 	cmd.data=STATUS_EXPLORING;
	// 	cmd_pub.publish(cmd);

	// }
	else if(status_flag==STATUS_CLOSE_TARGET)	//通过Action机制向着目标点前进
	{
		// 创建Action client
		MoveBaseClient act("move_base",true);

		while(!act.waitForServer(ros::Duration(5.0))){
			ROS_INFO("Waiting for the move_base action server to come up...");
		}

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = msg->header.frame_id;
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = msg->point.x;
		goal.target_pose.pose.position.y = msg->point.y;
		goal.target_pose.pose.position.z = msg->point.z;
		goal.target_pose.pose.orientation.w = 1;

		act.sendGoal(goal);
		ROS_INFO("Going to target: x:%0.6f, y:%0.6f, z:%0.6f", msg->point.x, msg->point.y, msg->point.z);
		act.waitForResult();

		status_flag==STATUS_EXPLORING;
	}
}

int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "move_to_target");

	// 创建节点句柄
	ros::NodeHandle n;
	// 一个节点可以根据需要设定若干个订阅或者发布者
    // 创建一个Subscriber，订阅名为/object_detect_pose，注册回调函数poseCallback
    ros::Subscriber pose_sub = n.subscribe("/object_detect_pose", 10, poseCallback);
	// 创建一个Publisher，发布名为cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	// 创建一个Publisher，发布名为/exploring_cmd的topic，消息类型为std_msgs::Int8，队列长度10
	cmd_pub = n.advertise<std_msgs::Int8>("/exploring_cmd", 10);

    // 循环等待回调函数
    ros::spin();

	return 0;
}