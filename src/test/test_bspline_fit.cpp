#include <trajectory_planner/bspline.h>


void test_fit(){
	Eigen::Vector3d p1 (0, 0, 1);
	Eigen::Vector3d p2 (0, 0.4, 1);
	Eigen::Vector3d p3 (0, 0.8, 1);
	Eigen::Vector3d p4 (0, 1.2, 1);
	Eigen::Vector3d p5 (0, 1.6, 1);
	Eigen::Vector3d p6 (0, 2.0, 1);
	Eigen::Vector3d p7 (0, 2.4, 1);
	Eigen::Vector3d p8 (0, 2.8, 1);
	Eigen::Vector3d p9 (0, 3.2, 1);
	Eigen::Vector3d p10 (0, 3.6, 1);

	std::vector<Eigen::Vector3d> points {p1,p2,p3,p4,p5,p6,p7,p8,p9,p10};
	Eigen::Vector3d startVel (0, 0 , 0);
	Eigen::Vector3d endVel (0, 0 , 0);
	Eigen::Vector3d startAcc (0, 0 , 0);
	Eigen::Vector3d endAcc (0, 0 , 0);
	std::vector<Eigen::Vector3d> startEndCondition {startVel, endVel, startAcc, endAcc};

	Eigen::MatrixXd controlPoints;
	double ts = 0.1;
	trajPlanner::bspline bs;
	for (int n=0; n<100; ++n){
		cout << "============================" << n  << "========================"<< endl;
		cout << "ts: " << ts << endl;
		bs.parameterizeToBspline(ts, points, startEndCondition, controlPoints);
		int numControlPoints = controlPoints.cols();
		for (int i=0; i<numControlPoints; ++i){
			Eigen::Vector3d pt = controlPoints.col(i);
			cout << "control point: " << i << ", " << pt.transpose() << endl;
		}
		bs = trajPlanner::bspline(3, controlPoints, ts);

		// ts += 0.01;
		points.clear();
		cout << "duration: " << bs.getDuration() << endl;
		for (double t=0; t<bs.getDuration(); t+=ts){
			cout << "fit point at " << t << " : " << bs.at(t).transpose() << endl;
			points.push_back(bs.at(t));
		}
		for (double t=0; t<bs.getDuration(); t+=0.1){
			cout << "fit point at wrt origin " << t << " : " << bs.at(t).transpose() << endl;
		}

	}
}


int main(int argc, char** argv){
	test_fit();
	return 0;
}