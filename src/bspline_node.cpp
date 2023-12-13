#include <ros/ros.h>
#include <map_manager/occupancyMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_planner/polyTrajOccMap.h>
#include <trajectory_planner/bsplineTraj.h>

using std::cout;
using std::endl;


bool firstTime = true;
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
ros::Publisher originalBsplinePub;
ros::Publisher originalCptsPub;
ros::Publisher inputTrajPub;
ros::Publisher inputTrajPointPub;
ros::Publisher originalCptsBeforePub;

bool initStart = false;
visualization_msgs::Marker startMarker;
bool initGoal = false;
visualization_msgs::Marker goalMarker;
Eigen::MatrixXd originCpts;
double controlPointTs;
nav_msgs::Path inputTraj;
Eigen::MatrixXd controlPointsBefore;


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



void publishTrajectory(const Eigen::MatrixXd& controlPoints, const ros::Publisher& cptPublisher, const ros::Publisher& trajPublisher, std::string ns, double r, double g, double b, double ts){
	if (controlPoints.cols() == 0) return;
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> pointVec;
	visualization_msgs::Marker point;
	int pointCount = 0;
	for (int i=0; i<controlPoints.cols(); ++i){
		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = ns;
		point.id = pointCount;
		point.type = visualization_msgs::Marker::SPHERE;
		point.action = visualization_msgs::Marker::ADD;
		point.pose.position.x = controlPoints(0, i);
		point.pose.position.y = controlPoints(1, i);
		point.pose.position.z = controlPoints(2, i);
		point.lifetime = ros::Duration(0.05);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 1.0;
		point.color.r = r;
		point.color.g = g;
		point.color.b = b;
		pointVec.push_back(point);
		++pointCount;			
	}
	msg.markers = pointVec;	
	cptPublisher.publish(msg);

	// trajctory
	nav_msgs::Path traj;
	Eigen::Vector3d p;
	geometry_msgs::PoseStamped ps;
	trajPlanner::bspline bsplineTraj = trajPlanner::bspline (3, controlPoints, ts);
	for (double t=0; t<=bsplineTraj.getDuration(); t+=0.1){
		p = bsplineTraj.at(t);
		ps.pose.position.x = p(0);
		ps.pose.position.y = p(1);
		ps.pose.position.z = p(2);
		traj.poses.push_back(ps);
	}
	traj.header.frame_id = "map";
	trajPublisher.publish(traj);
}

void publishPathMsg(const ros::Publisher& trajPub, nav_msgs::Path& traj){
	traj.header.frame_id = "map";
	trajPub.publish(traj);
}

void publishInputPath(const ros::Publisher& inputPathPub, const nav_msgs::Path& input, std::string ns, double r, double g, double b){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> pointVec;
	visualization_msgs::Marker point;
	int pointCount = 0;
	for (int i=0; i<int(input.poses.size()); ++i){
		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = ns;
		point.id = pointCount;
		point.type = visualization_msgs::Marker::SPHERE;
		point.action = visualization_msgs::Marker::ADD;
		point.pose.position.x = input.poses[i].pose.position.x;
		point.pose.position.y = input.poses[i].pose.position.y;
		point.pose.position.z = input.poses[i].pose.position.z;
		point.lifetime = ros::Duration(0.1);
		point.scale.x = 0.15;
		point.scale.y = 0.15;
		point.scale.z = 0.15;
		point.color.a = 0.7;
		point.color.r = r;
		point.color.g = g;
		point.color.b = b;
		pointVec.push_back(point);
		++pointCount;			
	}
	msg.markers = pointVec;
	inputPathPub.publish(msg);
}

void publishControlPoints(const Eigen::MatrixXd& controlPoints, const ros::Publisher& cptPublisher, std::string ns, double r, double g, double b){
	visualization_msgs::MarkerArray msg;
	std::vector<visualization_msgs::Marker> pointVec;
	visualization_msgs::Marker point;
	int pointCount = 0;
	for (int i=0; i<controlPoints.cols(); ++i){
		point.header.frame_id = "map";
		point.header.stamp = ros::Time::now();
		point.ns = ns;
		point.id = pointCount;
		point.type = visualization_msgs::Marker::SPHERE;
		point.action = visualization_msgs::Marker::ADD;
		point.pose.position.x = controlPoints(0, i);
		point.pose.position.y = controlPoints(1, i);
		point.pose.position.z = controlPoints(2, i);
		point.lifetime = ros::Duration(0.05);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 1.0;
		point.color.r = r;
		point.color.g = g;
		point.color.b = b;
		pointVec.push_back(point);
		++pointCount;			
	}
	msg.markers = pointVec;	
	cptPublisher.publish(msg);
}

void publishTraj(){
	ros::Rate r (30);
	while (ros::ok()){
		publishPathMsg(inputTrajPub, inputTraj);
		publishTrajectory(originCpts, originalCptsPub, originalBsplinePub, "origin", 0, 0, 1, controlPointTs);
		publishInputPath(inputTrajPointPub, inputTraj, "origin", 0, 0, 1);
		publishControlPoints(controlPointsBefore, originalCptsBeforePub, "origin_cpts_before", 0, 0, 1);
		r.sleep();
	}
}






int main(int argc, char** argv){
	ros::init(argc, argv, "bspline_navigation_node");
	ros::NodeHandle nh;

	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);
	startVisPub = nh.advertise<visualization_msgs::Marker>("/start_position", 1000);
	goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);
	originalBsplinePub = nh.advertise<nav_msgs::Path>("/original_trajectory", 1000);
	originalCptsPub = nh.advertise<visualization_msgs::MarkerArray>("/original_control_points", 1000);
	inputTrajPub = nh.advertise<nav_msgs::Path>("/input_traj", 1000);
	inputTrajPointPub = nh.advertise<visualization_msgs::MarkerArray>("/input_traj_point", 1000);
	originalCptsBeforePub =  nh.advertise<visualization_msgs::MarkerArray>("/control_points_before", 1000);


	std::thread startVisWorker_ = std::thread(publishStartVis);
	std::thread goalVisWorker_ = std::thread(publishGoalVis);
	std::thread trajVisWorker_ = std::thread(publishTraj);
	

	double desiredVel, desiredAcc;
	nh.getParam("desired_velocity", desiredVel);
	nh.getParam("desired_acceleration", desiredAcc);


	// map
	std::shared_ptr<mapManager::occMap> map;
	map.reset(new mapManager::occMap (nh));


	// min snap planner
	std::shared_ptr<trajPlanner::polyTrajOccMap> polyTraj;
	polyTraj.reset(new trajPlanner::polyTrajOccMap (nh));
	polyTraj->setMap(map);
	polyTraj->updateDesiredVel(desiredVel);
	polyTraj->updateDesiredAcc(desiredAcc);

	// bspline planner
	std::shared_ptr<trajPlanner::bsplineTraj> bsplineTraj;
	bsplineTraj.reset(new trajPlanner::bsplineTraj (nh));
	bsplineTraj->setMap(map);
	bsplineTraj->updateMaxVel(desiredVel);
	bsplineTraj->updateMaxAcc(desiredAcc);


	std::vector<double> start;
	std::vector<double> goal;
	std::vector<std::vector<double>> waypoints (2);
	int countLoop = 0;
	ros::Rate r(10);
	while (ros::ok()){
		cout << "----------------------------------------------------" << endl;
		cout << "[Planner Node]: Request No. " << countLoop+1 << endl;
		if (firstTime){
			cout << "[Planner Node]: Wait for start point..." << endl;
		}
		while (ros::ok()){
			if (newMsg){
				if (firstTime){
					start = newPoint;
					firstTime = false;
					newMsg = false;
				}
				else{
					start = goal;
				}
				
				cout << "[Planner Node]: start point OK. (" << start[0] << " " << start[1] << " " << start[2] << ")" << endl;
				waypoints[0] = start;
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
				goal = newPoint;

				newMsg = false;
				cout << "[Planner Node]: goal point OK. (" << goal[0] << " " << goal[1] << " " << goal[2] << ")" << endl;
				waypoints[1] = goal;
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

		std::vector<Eigen::Vector3d> startEndConditions;
		std::vector<Eigen::Vector3d> egoBsplineStartEndConditions;
		Eigen::Vector3d currVel (0.0, 0.0, 0.0);
		Eigen::Vector3d currAcc (0.0, 0.0, 0.0);
		Eigen::Vector3d endVel (0.0, 0.0, 0.0);
		Eigen::Vector3d endAcc (0.0, 0.0, 0.0);

		startEndConditions.push_back(currVel);
		startEndConditions.push_back(endVel);
		startEndConditions.push_back(currAcc);
		startEndConditions.push_back(endAcc);
		



		// min snap traj generation
		nav_msgs::Path waypointsMsg;
		geometry_msgs::PoseStamped pStart, pGoal;
		pStart.pose.position.x = waypoints[0][0];
		pStart.pose.position.y = waypoints[0][1];
		pStart.pose.position.z = waypoints[0][2];

		pGoal.pose.position.x = waypoints[1][0];
		pGoal.pose.position.y = waypoints[1][1];
		pGoal.pose.position.z = waypoints[1][2];
		waypointsMsg.poses = {pStart, pGoal};

		polyTraj->updatePath(waypointsMsg, startEndConditions);
		polyTraj->makePlan(false);

		// adjust input traj length
		nav_msgs::Path adjustedInputPolyTraj;
		bool satisfyDistanceCheck = false;
		double initTs = bsplineTraj->getInitTs();
		double dtTemp = initTs;
		double finalTimeTemp;
		ros::Time startTime = ros::Time::now();
		ros::Time currTime;
		while (ros::ok()){
			currTime = ros::Time::now();
			if ((currTime - startTime).toSec() >= 0.05){
				cout << "[AutoFlight]: Exceed path check time. Use the best." << endl;
				break;
			}
			nav_msgs::Path inputPolyTraj = polyTraj->getTrajectory(dtTemp);
			satisfyDistanceCheck = bsplineTraj->inputPathCheck(inputPolyTraj, adjustedInputPolyTraj, dtTemp, finalTimeTemp);
			if (satisfyDistanceCheck) break;
			
			dtTemp *= 0.8;
		}
		inputTraj = adjustedInputPolyTraj;


		// bspline trajectory
		bool updateSuccess = bsplineTraj->updatePath(inputTraj, startEndConditions);
		controlPointsBefore = bsplineTraj->getControlPoints();
		if (updateSuccess){
			bool planSuccess = bsplineTraj->makePlan();
			originCpts = bsplineTraj->getControlPoints();
			cout << "Original Plan success: " << planSuccess << endl;
		}
		controlPointTs = bsplineTraj->getControlPointTs();


		firstTime = true;
		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}
	return 0;
}