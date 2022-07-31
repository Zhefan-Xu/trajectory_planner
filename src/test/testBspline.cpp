#include <ros/ros.h>
#include <trajectory_planner/bspline.h>

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

}