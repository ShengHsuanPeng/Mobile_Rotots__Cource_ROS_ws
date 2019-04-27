#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "math.h"

const float PI = 3.14159265358979323;

geometry_msgs::Point subgoal_pos;

void GoalCallback(const geometry_msgs::PoseStamped &msg)
{
	geometry_msgs::Quaternion odom_quat;
	/*
	subgoal_pos.x = 1;			//The initial pose of the robot is the origin of ground coordinate.
	subgoal_pos.y = 0;
	subgoal_pos.z = PI/2;
	*/
	subgoal_pos.x = msg.pose.position.x;					//unit: m
	subgoal_pos.y = msg.pose.position.y;					//unit: m
	odom_quat = msg.pose.orientation;			//quaternoin

	subgoal_pos.z = tf::getYaw(odom_quat);			//theta is transformed from quaternion to rpy. unit: rad(-PI~PI)
	
	ROS_INFO("x: %.3f /t y: %.3f /t th: %.3f",subgoal_pos.x,subgoal_pos.y,subgoal_pos.z);
}

int main(int argc, char * argv[]) {

  ros::init(argc, argv, "sim_subgoal");
  ros::NodeHandle ns;
	
	ros::Publisher pub = ns.advertise<geometry_msgs::Point>("/subgoal_position", 10);
	ros::Subscriber sub = ns.subscribe("/move_base_simple/goal", 10, GoalCallback);
	
	ros::Rate rate(10);

	
  while (ros::ok()){
			
		ros::spinOnce();    // Allow processing of incoming messages
		
		pub.publish(subgoal_pos);

    rate.sleep();	
    }
	
	return 0;

}
