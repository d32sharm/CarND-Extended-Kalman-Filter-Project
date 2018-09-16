#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  x_ = VectorXd(4);
  P_ = MatrixXd(4,4);
  F_ = MatrixXd(4,4);
  // H_ will change based on whether laser or radar.
  // for laser (2,4) and radar (3,4)
  // we will initialize for the laser for now
  H_ = MatrixXd(2,4);
  // R_ will change based on whether laser or radar.
  // for laser (2,2) and for radar (3,3)
  // we will initialize for laser for now
  R_ = MatrixXd(2,2);
  Q_ = MatrixXd(4,4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  x_ = x_ + K * (z - H_ * x_);
  P_ = (MatrixXd::Identity(4,4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  VectorXd h = VectorXd(3);
  h(0) = sqrt(x*x + y*y); //rho
  h(1) = atan2(y, x); //theta
  h(2) = (x * vx + y * vy)/h(0); //rho_dot

  MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
  x_ = x_ + K * (z - h);
  P_ = (MatrixXd::Identity(4,4) - K * H_) * P_;
}
