#include <ros/ros.h>
#include <trajectory_planner/bsplineTraj.h>
#include <global_planner/rrtOctomap.h>
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
// ros::Publisher bslpineVisPub;
bool initStart = false;
visualization_msgs::Marker startMarker;
bool initGoal = false;
visualization_msgs::Marker goalMarker;
nav_msgs::Path bsplineFitPath;

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

// void publishBsplineFitTraj(){
// 	ros::Rate r (10);
// 	while (ros::ok()){
// 		if (initGoal){
// 			bslpineVisPub.publish(bsplineFitPath);
// 		}
// 		r.sleep();
// 	}
// }

int main(int argc, char** argv){
	ros::init(argc, argv, "test_bspline_node");
	ros::NodeHandle nh;
	cout << "test bspline" << endl;

	double timestep = 0.1;
	int degree = 3;
	Eigen::MatrixXd controlPoints;
	int size = 10;
	controlPoints.resize(3, size);
	for (int i=0; i<size; ++i){
		controlPoints(0, i) = i;
		controlPoints(1, i) = i;
		controlPoints(2, i) = i;
	}

	cout << "control points: " << endl;
	cout << controlPoints << endl;

	trajPlanner::bspline bs (degree, controlPoints, timestep);

	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);
	startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
	goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);
	// bslpineVisPub = nh.advertise<nav_msgs::Path>("/bspline_fit_curve", 1000);
	std::thread startVisWorker_ = std::thread(publishStartVis);
	std::thread goalVisWorker_ = std::thread(publishGoalVis);
	// std::thread bsplineVisWorker_ = std::thread(publishBsplineFitTraj);
	
	const int N = 3; // dimension
	globalPlanner::rrtOctomap<N> rrtplanner (nh);
	cout << rrtplanner << endl;

	trajPlanner::polyTrajOctomap polyPlanner (nh);
	cout << polyPlanner << endl;

	trajPlanner::bsplineTraj bt;
	bt.init(nh);
	std::vector<Eigen::Vector3d> startEndCondition; // dummy start end condition (all zeros)
	for (int i=0; i<4; ++i){
		Eigen::Vector3d con (0, 0, 0);
		startEndCondition.push_back(con);
	}

	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Planner Node]: Request No. " << countLoop+1 << endl;
		cout << "[Planner Node]: Wait for start point..." << endl;
		while (ros::ok()){
			if (newMsg){
				std::vector<double> start = newPoint;
				rrtplanner.updateStart(start);
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
				rrtplanner.updateGoal(goal);
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

		// Generate waypoint path
		nav_msgs::Path path;
		rrtplanner.makePlan(path);


		// generate trajectory:
		polyPlanner.updatePath(path);
		nav_msgs::Path trajectory;
		polyPlanner.makePlan(trajectory);
		double duration = polyPlanner.getDuration();
		cout << "[Planner Node]: Duration: " << duration << "s." << endl;


		// fit trajectory with bspline curve and display bspline curve
		// bt.updatePath(trajectory, startEndCondition);
		// bsplineFitPath = bt.evalTrajToMsg();
		// cout << "trajectory size: " << trajectory.poses.size() << endl;


		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}

}