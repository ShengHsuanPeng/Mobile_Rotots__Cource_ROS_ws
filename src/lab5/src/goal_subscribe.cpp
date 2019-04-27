#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"

double goal_x = 0.0;
double goal_y = 0.0;
double goal_th = 0.0;

const float PI = 3.14159265358979323;

geometry_msgs::Quaternion odom_quat;

ros::Subscriber sub;

void GoalCallback(const geometry_msgs::PoseStamped &msg)
{
	goal_x = msg.pose.position.x;					//unit: m
	goal_y = msg.pose.position.y;					//unit: m
	odom_quat = msg.pose.orientation;			//quaternoin

	goal_th = tf::getYaw(odom_quat);			//theta is transformed from quaternion to rpy. unit: rad(-PI~PI)
	
	ROS_INFO("x: %.3f /t y: %.3f /t th: %.3f",goal_x,goal_y,goal_th);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_subscribe");
 
  ros::NodeHandle n;
  
  ros::Rate r(100.0);
  while(n.ok()){
 
    ros::spinOnce();               // check for incoming messages
 
		sub = n.subscribe("/move_base_simple/goal", 10, GoalCallback);

    r.sleep();
  }
}
