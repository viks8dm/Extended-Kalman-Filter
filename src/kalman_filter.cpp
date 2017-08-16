#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#include <iostream>
#include <math.h>
#define EPS 0.0001

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/******************************************************/
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/******************************************************/
void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  // predict state & covariance
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = (F_ * P_ * Ft) + Q_;
}


/******************************************************/
/* This is update for lidar data only
* assumes linear formuation for H
*/
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - (H_ * x_);
    estimateKF_state(y);
}


/******************************************************/
/* This is update for RADAR data only
* requires conversion from polar to cartesian for H
*/
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    // Change F with Fj and H with Fj for EKF
    // F and H need to be calculated at every point in time

    // redefine state vector using rho, theta, rho_dot for radar
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    double rho = sqrt(px*px + py*py);
    if (rho<EPS) {
        rho = EPS;
    }
    double theta = atan2(py, px);
    double rho_dot = (px*vx + py*vy) / rho;

    VectorXd h_x = VectorXd(3);
    h_x << rho, theta, rho_dot;

    VectorXd y = z - h_x;

    // normalize y[1] (angle)
    if (y[1]>M_PI) { y[1] -= 2*M_PI; }
    if (y[1]<-M_PI) { y[1] += 2*M_PI; }

    estimateKF_state(y);
}


/******************************************************/
/* This function has been added to reduce common lines
* from update functions for lidar and radar
*/
void KalmanFilter::estimateKF_state(const VectorXd &y) {
    /**
     * This function updates the estimates using Kalman Filter
     * equations.
     */

    // compute new Kalman gain
    MatrixXd Ht = H_.transpose();
    MatrixXd S = (H_ * P_ * Ht) + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K_ = P_ * Ht * Si;

    // new estimate
    x_ = x_ + (K_ * y);
    int x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - (K_ * H_)) * P_;

}
