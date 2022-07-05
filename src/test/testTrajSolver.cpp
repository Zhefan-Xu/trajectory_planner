#include <ros/ros.h>
#include <trajectory_planner/polyTrajSolver.h>
#include <trajectory_planner/utils.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>


void trajMsgConverter(const std::vector<trajPlanner::pose>& trajectoryTemp, nav_msgs::Path& trajectory){
	std::vector<geometry_msgs::PoseStamped> trajVec;
	for (trajPlanner::pose pTemp: trajectoryTemp){
		geometry_msgs::PoseStamped ps;
		ps.header.stamp = ros::Time();
		ps.header.frame_id = "map";
		ps.pose.position.x = pTemp.x;
		ps.pose.position.y = pTemp.y;
		ps.pose.position.z = pTemp.z;

		geometry_msgs::Quaternion quat = trajPlanner::quaternion_from_rpy(0, 0, pTemp.yaw);
		ps.pose.orientation = quat;
		trajVec.push_back(ps);
	}
	trajectory.header.stamp = ros::Time();
	trajectory.header.frame_id = "map";
	trajectory.poses = trajVec;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "test_solver");
	ros::NodeHandle nh;
	cout << "[Test Solver]: test" << endl;
	std::vector<trajPlanner::pose> waypointPath;

	int numWaypoints;
	nh.getParam("waypoint_num", numWaypoints);
	cout << "[Test Solver]: Number of waypoints: " << numWaypoints << endl;

	std::vector<visualization_msgs::Marker> waypointVec;
	int waypointCount = 0;
	for (int i=1; i<=numWaypoints; ++i){
		std::string waypoint_name = "waypoint_" + std::to_string(i);
		std::vector<double> pVec;
		nh.getParam(waypoint_name, pVec);
		trajPlanner::pose p (pVec[0], pVec[1], pVec[2]);
		cout << "[Test Solver]: " << p << endl;
		waypointPath.push_back(p);

		// vis
		visualization_msgs::Marker waypoint;
		waypoint.header.frame_id = "map";
		waypoint.header.stamp = ros::Time();
		waypoint.ns = "waypoint";
		waypoint.id = waypointCount;
		waypoint.type = visualization_msgs::Marker::SPHERE;
		waypoint.action = visualization_msgs::Marker::ADD;
		waypoint.pose.position.x = p.x;
		waypoint.pose.position.y = p.y;
		waypoint.pose.position.z = p.z;
		waypoint.lifetime = ros::Duration(0.5);
		waypoint.scale.x = 0.3;
		waypoint.scale.y = 0.3;
		waypoint.scale.z = 0.3;
		waypoint.color.a = 0.5;
		waypoint.color.r = 0.7;
		waypoint.color.g = 1.0;
		waypoint.color.b = 0.0;
		waypointVec.push_back(waypoint);
		++waypointCount;
	}
	visualization_msgs::MarkerArray waypointMsg;
	waypointMsg.markers = waypointVec;

	int polyDegree = 6;
	int diffDegree = 3;
	int continuityDegree = 2;
	double desiredVel = 1.0;

	// initial condition
	double vx = -1.0;
	double vy = 0;
	double vz = 0;

	trajPlanner::polyTrajSolver solver (polyDegree, diffDegree, continuityDegree, desiredVel);
	solver.updateInitVel(vx, vy, vz);
	solver.updatePath(waypointPath);
	// solver.setCorridorConstraint(1.0, 10);
	solver.solve();

	std::vector<trajPlanner::pose> trajectory;
	solver.getTrajectory(trajectory, 0.1);

	nav_msgs::Path trajVis;
	trajMsgConverter(trajectory, trajVis);

	// visualization
	ros::Publisher trajVisPub = nh.advertise<nav_msgs::Path>("path_vis", 1);
	ros::Publisher wpVisPub = nh.advertise<visualization_msgs::MarkerArray>("waypoint", 1);
	ros::Rate r (10);
	while (ros::ok()){
		trajVisPub.publish(trajVis);
		wpVisPub.publish(waypointMsg);
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}