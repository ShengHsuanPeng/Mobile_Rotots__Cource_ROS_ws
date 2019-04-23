#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>

#define PI 3.14159

turtlesim::Pose pose;


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


int main(int argc, char **argv)
{
	double speed, angSpeed, angle, distance;
	geometry_msgs::Twist vel_msg;

	distance = 1.0;
	speed = 1.0;
	angSpeed = 30*2*PI/360;
	angle = 90*2*PI/360;

    // Initialize the node here
	ros::init(argc, argv, "turtle");
    ros::NodeHandle node;

	// publisher for turtle twist
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);

    // subscriber for the turtle pose
	ros::Subscriber sub = node.subscribe("/turtle1/pose", 5, poseCallback);

    // Set the publish rate here
	ros::Rate rate(10);
	int count = 0;
	turtlesim::Pose startpose;
	//startpose.x = pose.x;
	//startpose.y = pose.y;
	//startpose.theta = pose.theta;
	ros::Time t0 = ros::Time::now();
	ros::Time t1;
		
    while (ros::ok() && count <4) 
	{
        ros::spinOnce();    // Allow processing of incoming messages
		//printf("x:%lf, y:%lf, theta:%lf\n",pose.x,pose.y,pose.theta/PI*180);
		
		t1 = ros::Time::now();
		//ROS_INFO("%lf\n", t1.toSec()-t0.toSec());
		
		float passedTime = distance/speed;
		float passedAngTime = angle/angSpeed;
		if(t1.toSec() < t0.toSec()+passedTime)
		{
			vel_msg.linear.x = speed;
			vel_msg.angular.z = 0;
			pub.publish(vel_msg);
		}
		else if (t1.toSec() < t0.toSec()+passedTime+passedAngTime) {
			vel_msg.linear.x = 0;
			vel_msg.angular.z = angSpeed;
			pub.publish(vel_msg);
		}
		else
		{
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0;
			pub.publish(vel_msg);
			t0 = ros::Time::now();
			count++;
		}
		rate.sleep();
    }
	ros::shutdown();
	return 0;
}

