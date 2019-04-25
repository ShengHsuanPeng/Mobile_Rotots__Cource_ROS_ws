#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

#define PI 3.14159265358979323

ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber sub2;

using namespace std;

geometry_msgs::Twist command;

float x_now = -3;
float y_now =3;
float th_now=0;
float subgoal_x=-3;
float subgoal_y=3;

void move()
{
	ROS_INFO("MOVING!!");

	float error_distance = sqrt(pow((subgoal_x-x_now),2)+pow((subgoal_y-y_now),2));
	float error_orientation = atan2((subgoal_y-y_now),(subgoal_x-x_now))-th_now;

	float Kp_d = 1.5;
	float Kp_o = 0.5;
	
	if(error_orientation > PI)
		error_orientation = error_orientation - 2*PI;
	else if(error_orientation < -PI)
		error_orientation = error_orientation + 2*PI;

	if(error_orientation != 0)
		command.linear.x = (0.005/(abs(error_orientation)))*error_distance;
	else
		command.linear.x = Kp_d*error_distance;

	command.angular.z = Kp_o*error_orientation; 

	if(command.linear.x > 0.25)
		command.linear.x = 0.25;
	else if(command.linear.x < -0.25)
		command.linear.x = -0.25;

	if(command.angular.z > 0.5)
		command.angular.z = 0.5;
	else if(command.angular.z < -0.5)
		command.angular.z = -0.5;

	ROS_INFO("linear:%.2f",command.linear.x);
	ROS_INFO("angular:%.2f",command.angular.z);

}


void SubgoalCallback(const geometry_msgs::Point &msg)
{
	subgoal_x = msg.x;
	subgoal_y = msg.y;


	command.linear.x = 0;
	command.linear.y = 0;
	command.angular.z = 0;
	
}

void PoseCallback(const nav_msgs::Odometry &situation){

	//Subscribe robot position

	x_now = situation.pose.pose.position.x;
	y_now = situation.pose.pose.position.y;
	th_now = situation.pose.pose.orientation.z;
}



int main(int argc, char **argv)
{
 
    // Initialize the node here
	ros::init(argc, argv, "nav");
    	ros::NodeHandle node;

    // Write a publisher for the robot movement here
	pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
    // Write a subscriber for the robot pose
	sub = node.subscribe("subgoal_position", 10, SubgoalCallback);
	sub2 = node.subscribe("robot_pose", 10, PoseCallback);
	
    // Set the publish rate here
	ros::Rate rate(100);
  
    while (ros::ok()) 
    {			
        ros::spinOnce();    // Allow processing of incoming messages
	move();
		pub.publish(command);

	
        rate.sleep();	
    }
	
	return 0;
}
