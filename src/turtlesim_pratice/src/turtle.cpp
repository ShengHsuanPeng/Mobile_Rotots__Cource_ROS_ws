#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <math.h>

#define PI 			3.14159
#define angle   	90*2*PI/360.0
#define distance	3.0


turtlesim::Pose pose;

enum TurtleStatus{
	INIT = 0,
	GO,
	REACHED, 
	STOP,
};
enum Direction { STRAIGHT=0, TURN_LEFT };


void poseCallback(const turtlesim::PoseConstPtr& msg)
{
	//Do your callback function here.
	//In this callback function, you need to write something
	//to make the turtle move with square trajectory.

	pose.x = msg->x;
	pose.y = msg->y;
	pose.theta = msg->theta;
	return;
}

float l2_distance(turtlesim::Pose goalPos, turtlesim::Pose startPos)
{
	float num = sqrt(pow(goalPos.x-startPos.x,2.0)+pow(goalPos.y-startPos.y,2.0));
	return num;
} 

int main(int argc, char **argv)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	turtlesim::Pose startPose;
	startPose.x = 5.544445;
	startPose.y = 5.544445;
	startPose.theta = 0;
	pose = startPose;

	turtlesim::Pose goal_pose;
	TurtleStatus status = INIT;
	Direction direction;

    // Initialize the node here
	ros::init(argc, argv, "turtle");
    ros::NodeHandle node;

	// publisher for turtle twist
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

    // subscriber for the turtle pose
	ros::Subscriber sub = node.subscribe("/turtle1/pose", 1000, &poseCallback);

    // Set the publish rate here
	ros::Rate rate(10);
	

    while (ros::ok()) 
	{
        ros::spinOnce();    // Allow processing of incoming messages
		switch (status)
		{
			case INIT:
				goal_pose.x = pose.x + distance*cos(pose.theta);
				goal_pose.y = pose.y + distance*sin(pose.theta);
				goal_pose.theta = pose.theta;
				direction = STRAIGHT;
				ROS_INFO("INIT %lf,%lf", goal_pose.x,goal_pose.y);
				status = GO;
				break;
			case GO:
				if(direction == STRAIGHT)
				{
					float dL = l2_distance(goal_pose, pose);
					ROS_INFO("GO STARIGHT: %f", dL);
					if(dL > 0.0001)
					{
						vel_msg.linear.x =  dL * 5;
					}
					else
					{
						vel_msg.linear.x = 0;
						status = REACHED;
						direction = TURN_LEFT;
					}
				}
				else
				{
					float dTheta = (goal_pose.theta - pose.theta);
					ROS_INFO("TURN: %f",dTheta);
					if(fabs(dTheta) > 0.0001)
					{
						vel_msg.angular.z =  dTheta * 5;
					}
					else
					{
						vel_msg.angular.z = 0;
						if(fabs(l2_distance(startPose,pose))>0.1)
						{
							status = REACHED;
							direction = STRAIGHT;
						}
						else
						{
							status = STOP;
						}
					}
				}
				pub.publish(vel_msg);
				break;
			case REACHED:
				if(direction == STRAIGHT)
				{
					goal_pose.x = pose.x + distance*cos(pose.theta);
					goal_pose.y = pose.y + distance*sin(pose.theta);
					goal_pose.theta = pose.theta;
				}
				else
				{
					goal_pose.x = pose.x;
					goal_pose.y = pose.y;
					goal_pose.theta = pose.theta+angle;
				}
				status = GO;
				break;
			case STOP:
				ROS_INFO("X%f, Y%f, Theta%f", pose.x, pose.y, pose.theta/PI*180);
				ros::shutdown();
				break;
			default:
				ROS_INFO("DEFAULT");
				break;
		}
		rate.sleep();
    }
	return 0;
}

