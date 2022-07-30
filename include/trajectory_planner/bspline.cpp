/*
	FILE: bspline.cpp
	------------------------
	function definition of basic bspline
*/

#include <trajectory_planner/bspline.h>

namespace trajPlanner{
	bspline::bspline(){}

	bspline::bspline(int degree, const Eigen::MatrixXd& controlPoints, double ts){
		this->degree_ = degree;
		this->controlPoints_ = controlPoints;
		this->ts_ = ts;
		this->initKnots();
	}

	void bspline::initKnots(){
		int controlPointsNum = this->controlPoints_.cols();
		int knotsNum = controlPointsNum-1 + this->degree+1 + 1; // t0 to tN + degree+1. total N+1 control points
		this->knots_.resize(knotsNum);

		for (int i=0; i<knotsNum; ++i){
			this->knots_(i) = (i - this->degree_) * this->ts_;
		}
	}


	void bspline::updateBsplineDegree(int degree){
		this->degree_ = degree;
	}

	void bspline::updateControlPoints(const Eigen::MatrixXd& controlPoints){
		this->controlPoints_ = controlPoints;
	}

	void bspline::updateTimestep(double ts){
		this->ts_ = ts;
	}


}