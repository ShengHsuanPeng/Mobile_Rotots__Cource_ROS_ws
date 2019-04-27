#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "math.h"
#include "stdlib.h"
#include <cmath>
#include <queue>
#include <vector>
#include <list>
#include <algorithm>
using namespace std;

float x_now = -3;
float y_now = 3;
float th_now = 0;
float x_goal = -3;	//default
float y_goal = 3;
float th_goal;
bool pos_rec = 0;
bool map_rec = 0;
int width;
int height;


int* map_data;

geometry_msgs::Point subgoal_pos;



struct position
{
	float x;
	float y;

	bool operator==(position a) const{
		if(fabs(a.x-x)<0.05 && fabs(a.y-y)<0.05)return true;
		else return false;
	}
};

int HeuristicCost(position pos) {
	return (int)round(100.0*(fabs(pos.x-x_goal)+fabs(pos.y-y_goal)));
}

struct map_astar // map information for A* 
{
	position pos;
	vector<position> action;
	int g_cost;

	bool operator<(const map_astar &o) const {
        return (g_cost+HeuristicCost(pos)) > (o.g_cost+HeuristicCost(o.pos));
    }
};

struct map_info // map_info
{
	int width;
	int height;
	float orign_x;
	float orign_y;
	float resolution;
};

struct succesor 
{
	position pos;
	int cost;
};

void Mapcallback(const nav_msgs::OccupancyGrid &msg) {
	
	//Subscribe Map information
	
	map_info map;

	map.width = msg.info.width;
	map.height = msg.info.height;
	map.orign_x = msg.info.origin.position.x;
	map.orign_y = msg.info.origin.position.y;
	map.resolution = msg.info.resolution;
	
	map_data = new int[map.width*map.height]();

	for(int i = 0; i < map.height; i++){
		for(int j = 0; j < map.width; j++){
			map_data[i*map.width + j] = msg.data[i*map.width + j];
		}
	}
	width = map.width;
	height = map.height;

	map_rec = true;
}


void Posecallback(const nav_msgs::Odometry &situation){

	//Subscribe robot position

	x_now = situation.pose.pose.position.x;
	y_now = situation.pose.pose.position.y;
	th_now = situation.pose.pose.orientation.z;
	
	pos_rec = true;
}


vector<succesor> GetSuccessors(position pos) {
	vector<succesor> successors;//ROS_INFO("107");
	int X = (int)round(pos.x*10.0+50.0);
	int Y = (int)round(pos.y*10.0+50.0);
	int margin = 1;
	//ROS_INFO("Getting x%lf, y%lf",pos.x,pos.y);
	for (int i=-margin+Y;i<=margin+Y;i++) {
		for (int j=-margin+X;j<=margin+X;j++) {
			if (abs(i-Y)!=margin && abs(j-X)!=margin)
				continue;
			if (i<0||i>height-1||j<0||j>width-1) 
				continue;
			if (map_data[i*width + j] == 0) {
				int cost=0;
				if(i==Y || j==X){
					cost = 10;
				} else {
					cost = 14;
				}
				position nodePos = {(j/10.0-5.0),(i/10.0-5.0)};
				succesor node = {nodePos, cost};
				//ROS_INFO("NEW SUCC x%lf, y%lf",nodePos.x,nodePos.y);
				successors.push_back(node);
			}
		}
	}
	return successors;
}


int ManhattanDist(int X, int Y, int X_end, int Y_end) {
	return round(100.0*(fabs(X-X_end)+fabs(Y-Y_end)));
}

vector<position> AStar() {
	priority_queue<map_astar> open_set;
	vector<position> closed_set;
	
	position start_state = {x_now, y_now};
	position goal_state = {x_goal, y_goal};
	if (start_state == goal_state) {
		return vector<position>();
	} 

    vector<position> action;
	map_astar node = {start_state, vector<position>(), 0};
	open_set.push(node);

	while (!open_set.empty()) {
		node = open_set.top();
        open_set.pop();
		position pos = node.pos;
		action = node.action;
		int g_cost = node.g_cost;

		if (find(closed_set.begin(), closed_set.end(),pos)!=closed_set.end()) {
			continue;
		}

		if (pos == goal_state) {
			ROS_INFO("Path Found");
			return action;
		} else {
			closed_set.push_back(pos);
		}
		
		vector<succesor> next_successors = GetSuccessors(pos);
		for (int i = 0; i < next_successors.size(); i++) {
			vector<position> new_action = action;
			new_action.push_back(next_successors[i].pos);
			
			map_astar new_node = 
				{next_successors[i].pos, new_action, g_cost + next_successors[i].cost};

			open_set.push(new_node);
		}
	}
	ROS_INFO("170");
	return vector<position>();
}
	
int main(int argc, char * argv[]) {
	ros::init(argc, argv, "map_listener");
	ros::NodeHandle ns;
	
	ros::Publisher pub = ns.advertise<geometry_msgs::Point>("/subgoal_position", 10);
	ros::Subscriber sub = ns.subscribe("/map", 1, Mapcallback);
	ros::Subscriber sub2 = ns.subscribe("/robot_pose", 10, Posecallback);
	x_now = -3;
	y_now = 3;
	pos_rec=true;
	
	ros::Rate rate(10);
	
	list<position> goals;
	position goal1={3.0,4.0}, goal2={3.0,-2.0}, goal3={-4.0,-3.0}, goal4={-3.0,-3.0};
	goals.push_back(goal1);
	goals.push_back(goal2);
	goals.push_back(goal3);
	goals.push_back(goal4);
	vector<position> action;
	int step=0;

	while (ros::ok()){
			
		ros::spinOnce();    // Allow processing of incoming messages

		if (pos_rec && map_rec) {
			position current = {x_now, y_now};
			position goal_state = {x_goal, y_goal};
			if(current == goal_state) {
				if (!goals.empty()) {
					x_goal = goals.front().x;
					y_goal = goals.front().y;

					ROS_INFO("NEW Target x%lf, y%lf",x_goal,y_goal);
					goals.pop_front();
				}
			}
			action = AStar();
			if(action.empty()) {
				ROS_INFO("No Path");
				continue;
			}
			
			position pos = action[0];
			
			subgoal_pos.x = pos.x;
			subgoal_pos.y = pos.y;
			ROS_INFO("go to x%.1lf y%.1lf, goal%.1lf %.1lf, now%.3lf %.3lf", subgoal_pos.x,subgoal_pos.y, x_goal,y_goal,x_now,y_now);

		}

		pub.publish(subgoal_pos);

		rate.sleep();	
	}
	
	return 0;

}
