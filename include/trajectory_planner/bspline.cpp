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
		int knotsNum = controlPointsNum-1 + this->degree_+1 + 1; // t0 to tN + degree+1. total N+1 control points
		this->knots_.resize(knotsNum);

		for (int i=0; i<knotsNum; ++i){
			this->knots_(i) = (i - this->degree_) * this->ts_;
		}
		this->duration_ = this->knots_(knotsNum - this->degree_ - 1);
	}



	Eigen::VectorXd bspline::at(double t){
		double tBounded = std::min(std::max(0.0, t), this->duration_);
		
		// determine the range of t in the knots
		int k = this->degree_;
		while (true){
			if (this->knots_(k+1) >= tBounded){
				break;
			}
			++k;
		}

		// deBoor's alg
		std::vector<Eigen::VectorXd> d;
		for (int i=0; i<=this->degree_; ++i){
			d.push_back(this->controlPoints_.col(k-this->degree_+i));
		}	

		for (int r=1; r<=this->degree_; ++r){
			for (int i=this->degree_; i>=r; --i){
				double alpha = (tBounded - this->knots_[i + k - this->degree_])/(this->knots_[i + 1 + k - r] - this->knots_[i + k - this->degree_]);
				d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
			}
		}

		return d[this->degree_];
	}

	double bspline::getDuration(){
		return this->duration_;
	}

	bspline bspline::getDerivative(){
    	Eigen::MatrixXd ctp(this->controlPoints_.rows(), this->controlPoints_.cols() - 1);
    	for (int i=0; i<ctp.cols(); ++i){
			ctp.col(i) = this->degree_ * (this->controlPoints_.col(i + 1) - this->controlPoints_.col(i)) / (this->knots_(i + this->degree_ + 1) - this->knots_(i + 1));
    	}

    	bspline derivative (this->degree_-1, ctp, this->ts_);
    	return derivative;
	}

	void bspline::parameterizeToBspline(double ts, 
										const std::vector<Eigen::Vector3d>& points, 
										const std::vector<Eigen::Vector3d>& startEndConditions,
										Eigen::MatrixXd& controlPoints){
		if (ts <= 0){
			cout << "[Bspline]: Invalid timestep." << endl;
			exit(0);
		}

		if (points.size() <= 3){
			cout << "[Bspline]: Point set only has " << points.size() << " points. At least need 4." << endl;
			exit(0);
			return;
		}

		if (startEndConditions.size() != 4){
			cout << "[Bspline]: Please enter correct start and end acc/vel." << endl;
			exit(0);
			return;
		}

	    int K = points.size();

	    // write A
	    Eigen::Vector3d prow(3), vrow(3), arow(3);
	    prow << 1, 4, 1;
	    vrow << -1, 0, 1;
	    arow << 1, -2, 1;

		Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K+4, K+2);

		for (int i=0; i<K; ++i){
			A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();
		}

		A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
		A.block(K+1, K-1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
		A.block(K+2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
		A.block(K+3, K-1, 1, 3) = (1 / ts / ts) * arow.transpose();

		// write b
		Eigen::VectorXd bx(K+4), by(K+4), bz(K+4);
		for (int i=0; i<K; ++i){
			bx(i) = points[i](0);
			by(i) = points[i](1);
			bz(i) = points[i](2);
		}

		for (int i=0; i<4; ++i){
			bx(K+i) = startEndConditions[i](0);
			by(K+i) = startEndConditions[i](1);
			bz(K+i) = startEndConditions[i](2);
		}

		// solve Ax = b
		Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
		Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
		Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

		// convert to control points
	    controlPoints.resize(3, K+2);
	    controlPoints.row(0) = px.transpose();
	    controlPoints.row(1) = py.transpose();
	    controlPoints.row(2) = pz.transpose();
	}


	Eigen::MatrixXd bspline::getControlPoints(){
		return this->controlPoints_;
	}
}