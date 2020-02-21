#include "kalman_filter.h"
#include "tools.h"
#include <math.h>
#include <iostream>
#define pi 3.1415926

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

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
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd Y_ = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K_ = PHt * Si_;
  //new estimation
  x_ = x_ + (K_ * Y_);
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size); // Define the identity matrix
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  float pho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float phi = atan2(x_(1),x_(0));
  float pho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/pho;
  VectorXd h_x_= VectorXd(3);
  h_x_<<pho, phi, pho_dot; 
  VectorXd Y_ = z - h_x_;
  //Normalizing the angle in vector Y_
  if(Y_(1)<-pi)
  {
    Y_(1) += 2 * pi;
   }
  else if(Y_(1)>pi)
  {
    Y_(1) -= 2 * pi;
   }
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_ * P_* Ht + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K_ = PHt * Si_;
  // new estimate
  x_ = x_ + K_ * Y_;
  long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(x_size, x_size); // Define the identity matrix
  P_ = (I_ - K_ * H_) * P_;
}
