#include "kalman_filter.h"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  MatrixXd I_ << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
  MatrixXd Y_;
  MatrixXd S_;
  MatrixXd K_;
  Tools tools;
  
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  Y_ = z - H_ * x_;
  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * Y_;
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float sqrt_px_2_py_2 = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  VectorXd h_x_<<sqrt_px_2_py_2; atan2(x_(1),x_(0)); (x_(0)*x_(2)+x_(1)*x_(3))/sqrt_px_2_py_2; 
  Hj_ = tools.CalculateJacobian(x_);
  Y_ = z - h_x_;
  S_ = Hj_*P_*Hj_.transpose() + R_;
  K_ = P_ * Hj_.transpose() * S_.inverse();
  x_ = x_ + K_ * Y_;
  P_ = (I_ - K_ * Hj_) * P_;
  
}
