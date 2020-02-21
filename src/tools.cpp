#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd sum(4);
  sum <<0,0,0,0;
  VectorXd RMSE(4);
  RMSE <<0,0,0,0;
  if(estimations.size()==0||(estimations.size()!=ground_truth.size())) //Check the size of the estimations
  {
    std::cout<<"ERROR: The size of the estimations is incorrect\n";
   }
  else
  {
    for(int i=0;i<estimations.size();++i)
    {
      VectorXd residual = estimations[i] - ground_truth[i];
      VectorXd square = residual.array()*residual.array();
      sum += square;
     }
    RMSE = (sum/estimations.size()).array().sqrt();
  }
  return RMSE;
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  float px_2 = px * px;
  float py_2 = py * py;
  float sqrt_px_2_py_2 = sqrt(px_2+py_2);
  float pow_sqrt_px_2_py_2 = pow(sqrt_px_2_py_2,3);

  if(px==0 && py==0) //Check if the equation in the Jacobian matrix will be divided by 0
  {
    std::cout<<"ERROR: The equation in the Jacobian matrix is divided by 0(px and py =0)\n";
   }
  else
  {
    Hj<<px/sqrt_px_2_py_2 ,py/sqrt_px_2_py_2 ,0,0,
       -py/(px_2+py_2),px/(px_2+py_2),0,0,
    py*(vx*py-vy*px)/pow_sqrt_px_2_py_2,px*(vy*px-vx*py)/pow_sqrt_px_2_py_2,px/sqrt_px_2_py_2,py/sqrt_px_2_py_2;
   }
  return Hj;
}
