/*
*	File: poly_RRTStar_node.cpp
*	---------------
*   polynomial trajectory with RRT* for two points
*/
#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_planner/polyTrajOctomap.h>

using std::cout;
using std::endl;


bool newMsg = false;
std::vector<double> newPoint {0, 0, 1.0};
void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
	newPoint[0] = cp->pose.position.x;
	newPoint[1] = cp->pose.position.y;
	newPoint[2] = 1.0; // set height to be 1.0 m
	newMsg = true;
}

ros::Publisher startVisPub;
ros::Publisher goalVisPub;
bool initStart = false;
visualization_msgs::Marker startMarker;
bool initGoal = false;
visualization_msgs::Marker goalMarker;

void publishStartVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (initStart){
			startVisPub.publish(startMarker);
		}
		r.sleep();
	}
}

void publishGoalVis(){
	ros::Rate r(10);
	while (ros::ok()){
		if (initGoal){
			goalVisPub.publish(goalMarker);
		}
		r.sleep();
	}
}	


int main(int argc, char** argv){
	ros::init(argc, argv, "Poly_RRT*_test_node");
	ros::NodeHandle nh;

	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);
	startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
	goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);
	std::thread startVisWorker_ = std::thread(publishStartVis);
	std::thread goalVisWorker_ = std::thread(publishGoalVis);
	
	const int N = 3; // dimension
	globalPlanner::rrtStarOctomap<N> rrtStarPlanner (nh);
	cout << rrtStarPlanner << endl;

	trajPlanner::polyTrajOctomap polyPlanner (nh);
	cout << polyPlanner << endl;

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Planner Node]: Request No. " << countLoop+1 << endl;
		cout << "[Planner Node]: Wait for start point..." << endl;
		while (ros::ok()){
			if (newMsg){
				std::vector<double> start = newPoint;
				rrtStarPlanner.updateStart(start);
				newMsg = false;
				cout << "[Planner Node]: start point OK. (" << start[0] << " " << start[1] << " " << start[2] << ")" << endl;
				
				// visualization:
				initStart = true;
				initGoal = false;
				startMarker.header.frame_id = "map";
				startMarker.header.stamp = ros::Time();
				startMarker.ns = "start_vis";
				startMarker.id = 0;
				startMarker.type = visualization_msgs::Marker::SPHERE;
				startMarker.action = visualization_msgs::Marker::ADD;
				startMarker.pose.position.x = start[0];
				startMarker.pose.position.y = start[1];
				startMarker.pose.position.z = start[2];
				startMarker.lifetime = ros::Duration(0.5);
				startMarker.scale.x = 0.4;
				startMarker.scale.y = 0.4;
				startMarker.scale.z = 0.4;
				startMarker.color.a = 0.7;
				startMarker.color.r = 1.0;
				startMarker.color.g = 0.5;
				startMarker.color.b = 1.0;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		cout << "[Planner Node]: Wait for goal point..." << endl;
		while (ros::ok()){
			if (newMsg){
				std::vector<double> goal = newPoint;
				rrtStarPlanner.updateGoal(goal);
				newMsg = false;
				cout << "[Planner Node]: goal point OK. (" << goal[0] << " " << goal[1] << " " << goal[2] << ")" << endl;
				
				// visualizaiton:
				initGoal = true;
				goalMarker.header.frame_id = "map";
				goalMarker.header.stamp = ros::Time();
				goalMarker.ns = "goal_vis";
				goalMarker.id = 0;
				goalMarker.type = visualization_msgs::Marker::SPHERE;
				goalMarker.action = visualization_msgs::Marker::ADD;
				goalMarker.pose.position.x = goal[0];
				goalMarker.pose.position.y = goal[1];
				goalMarker.pose.position.z = goal[2];
				goalMarker.lifetime = ros::Duration(0.5);
				goalMarker.scale.x = 0.4;
				goalMarker.scale.y = 0.4;
				goalMarker.scale.z = 0.4;
				goalMarker.color.a = 0.7;
				goalMarker.color.r = 0.2;
				goalMarker.color.g = 1.0;
				goalMarker.color.b = 0.2;
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		// generate waypoint path
		nav_msgs::Path path;
		rrtStarPlanner.makePlan(path);


		// generate trajectory:
		polyPlanner.updatePath(path);
		polyPlanner.makePlan();
		double duration = polyPlanner.getDuration();
		cout << "[Planner Node]: Duration: " << duration << "s." << endl;


		// visualization
		// ros::Time startTime = ros::Time::now();
		// ros::Time currTime = ros::Time::now();
		// ros::Rate r(50);
		// double dt = (currTime - startTime).toSec();
		// while (dt <= duration){
		// 	currTime = ros::Time::now();
		// 	dt = (currTime - startTime).toSec();
		// 	if (dt > duration){
		// 		break;
		// 	}
		// 	geometry_msgs::PoseStamped p = polyPlanner.getPose(dt);
		// 	posePub.publish(p);
		// 	r.sleep();
		// }

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}
	return 0;
}