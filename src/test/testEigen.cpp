#include <OsqpEigen/OsqpEigen.h>

using std::cout; using std::endl;

int main(int argc, char** argv){
	Eigen::VectorXd v1;
	v1.resize(5);
	v1(0) = 1.0;
	v1(1) = 2.0;
	v1(2) = 3.0;
	v1(3) = 4.0;
	v1(4) = 5.0;
	// v.resize()

	Eigen::MatrixXd m1;
	m1 = v1 * v1.transpose();
	cout << m1 << endl;
	cout << v1 * v1.transpose() << endl;
	cout << v1 << endl;
	return 0;
}