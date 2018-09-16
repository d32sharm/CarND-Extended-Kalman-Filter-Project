#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse = VectorXd(4);
  rmse << 0, 0, 0, 0;

  for (int i = 0; i < estimations.size(); ++i)
  {
  	VectorXd error = estimations[i] - ground_truth[i];
  	for (int j = 0; j < error.size(); ++j)
	{
	  rmse(j) += error(j) * error(j);
	}
  }

  for (int i = 0; i < rmse.size(); ++i)
  {
  	rmse(i) = sqrt(rmse(i));
  }	

  return rmse/estimations.size();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd H = MatrixXd(3,4);
  float x = x_state(0);
  float y = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  H << x/sqrt(x*x + y*y), y/sqrt(x*x + y*y), 0, 0,
  	   -y/(x*x + y*y), x/(x*x + y*y), 0, 0,
  	   ((x*x + y*y)*vx + (x*vx + y*vy)*x)/(sqrt(x*x + y*y) * (x*x + y*y)), 
  	   ((x*x + y*y)*vy + (x*vx + y*vy)*y)/(sqrt(x*x + y*y) * (x*x + y*y)),
  	   x/sqrt(x*x + y*y), y/sqrt(x*x + y*y);

  return H;
}
