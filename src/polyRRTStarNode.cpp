#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <global_planner/utils.h>
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

int main(int argc, char** argv){
	ros::init(argc, argv, "RRT_test_node");
	ros::NodeHandle nh;

	// subscriber for clicked start and goal:
	ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);

	

	const int N = 3; // dimension
	std::vector<double> collisionBox, envBox;
	double delQ, dR, connectGoalRatio, mapRes, timeout, rNeighborhood, maxNeighbors, degree, continuityDegree, veld, regularizationWeights, initR, fs, delT;;
	int diffDegree, maxIter;
	bool visRRT, visPath;
	
	nh.getParam("/collision_box", collisionBox);
	nh.getParam("/env_box", envBox);
	nh.getParam("/rrt_incremental_distance", delQ);
	nh.getParam("/rrt_incremental_distance", connectGoalRatio);
	nh.getParam("/goal_reach_distance", dR);
	nh.getParam("/map_resolution", mapRes);
	nh.getParam("/timeout", timeout);
	nh.getParam("/vis_path", visPath);
	nh.getParam("/neighborhood_radius", rNeighborhood);
	nh.getParam("/max_num_neighbors", maxNeighbors);


	
	globalPlanner::rrtStarOctomap<N> rrtStarPlanner (nh, rNeighborhood, maxNeighbors, collisionBox, envBox, mapRes, delQ, dR, connectGoalRatio, timeout, visPath);
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
				break;
			}
			ros::spinOnce();
			r.sleep();
		}

		
		std::vector<KDTree::Point<N>> plan;
		rrtStarPlanner.makePlan(plan);

		// conversion:
		std::vector<trajPlanner::pose> path;
		trajPlanner::convertPointPlan(plan, path);

		// generate trajectory:
		polyPlanner.updatePath(path);
		polyPlanner.makePlan();
		double duration = polyPlanner.getDuration();
		cout << "[Planner Node]: Duration: " << duration << "s." << endl;

		ros::Time startTime = ros::Time::now();
		ros::Time currTime = ros::Time::now();
		ros::Rate r(50);
		double dt = (currTime - startTime).toSec();
		while (dt <= duration){
			currTime = ros::Time::now();
			dt = (currTime - startTime).toSec();
			geometry_msgs::PoseStamped p = polyPlanner.getPose(dt);
			posePub.publish(p);
			r.sleep();
		}

		++countLoop;
		cout << "----------------------------------------------------" << endl;
	}
	return 0;
}