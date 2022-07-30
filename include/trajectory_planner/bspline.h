/*
	FILE: bspline.h
	--------------------------------
	header files for basic bspline operation
*/

#ifndef BSPLINE_H
#define BSPLINE_H

#include <Eigen/Eigen>


namespace trajPlanner{
	class bspline{
	private:
		int degree_; // polynomial degree
		Eigen::MatrixXd controlPoints_;
		double ts_; // timestep
		Eigen::VectorXd knots_; // time knots

	public:
		bspline();
		bspline(int degree, const Eigen::MatrixXd& controlPoints, double ts);
		void initKnots();

		void updateBsplineDegree(int degree);
		void updateControlPoints(const Eigen::MatrixXd& controlPoints);
		void updateTimeStep(double ts);
	};
}

#endif