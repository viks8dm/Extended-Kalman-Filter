#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
#define EPS 0.0001

Tools::Tools() {}

Tools::~Tools() {}

/******************************************************/
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4), diff(4), diff_sq(4), sum(4);
    // initialize vectors
    rmse << 0,0,0,0;
    
    // check estimation and ground-truth size
    if ((estimations.size() == 0) || (estimations.size() != ground_truth.size())) {
        cout << "ERROR: invalid estimations vector or ground_truth vector" << endl;

        if (estimations.size() == 0)
            cout << "ERROR-Type: estimations vector size = 0" << endl;
        else
            cout << "ERROR-Type: estimations-vector & ground_truth-vector size mismatch" << endl;

        return rmse;
    }

    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        diff = estimations[i] - ground_truth[i];
        diff_sq = diff.array()*diff.array();
        rmse = rmse + diff_sq;
    }

    //calculate the mean
    rmse = rmse / estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}


/******************************************************/
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //RMSE jumps when px or py are very small, hence add check to address this
    if ((fabs(px) < EPS) && (fabs(py) < EPS)) {
        px = EPS;
        py = EPS;
    }
    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    /* RMSE jumps when px or py are very small, hence add check to address
     * this division by zero issue
     */
    if (fabs(c1) < (EPS*EPS)) {
        c1 = (EPS*EPS);
    }
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
