#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

// 2*Pi
double pi2 = M_PI * 2;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;
	
	if (estimations.size() != ground_truth.size() || 
		estimations.size() == 0) {
		cout << "invalid estimation or ground truth data" << endl;
		return rmse;
	}

	for (auto i = 0; i < estimations.size(); i++) {
		VectorXd residual = estimations[i] - ground_truth[i];
		// coef-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	// calc mean
	rmse = rmse / estimations.size();
	// calc square root
	return rmse.array().sqrt();
}

// Normalize angle to be between Pi and -Pi
float Tools::normalize_angle(float x) {
    if (x > M_PI) {
        x = fmod(x, pi2);
        if (x > M_PI)
            x -= pi2;
    }
    if (x < -M_PI) {
        x = fmod(x, -pi2);
        if (x < -M_PI)
            x += pi2;
    }
	return x;
}
