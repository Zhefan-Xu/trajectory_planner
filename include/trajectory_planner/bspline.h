/*
	FILE: bspline.h
	--------------------------------
	header files for basic bspline operation
*/

#ifndef BSPLINE_H
#define BSPLINE_H
#include <iostream>
#include <Eigen/Eigen>

using std::cout; using std::endl;
namespace trajPlanner{
	class bspline{
	private:
		int degree_; // polynomial degree
		Eigen::MatrixXd controlPoints_;
		double ts_; // timestep
		Eigen::VectorXd knots_; // time knots
		double duration_; // duration of the whole bspline

	public:
		bspline();
		bspline(int degree, const Eigen::MatrixXd& controlPoints, double ts);
		void initKnots();
		Eigen::VectorXd at(double t);
		double getDuration();
		bspline getDerivative();

		static void parameterizeToBspline(double ts, 
										  const std::vector<Eigen::Vector3d>& points, 
										  const std::vector<Eigen::Vector3d>& startEndConditions,
										  Eigen::MatrixXd& controlPoints); // static function to fit a curve with bspine
		Eigen::MatrixXd getControlPoints();
	};
}

#endif