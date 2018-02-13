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
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
    * Assume x_state has 4 states (2d position and 2d velocity)
  */
  MatrixXd H_(3, 4);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double r2 = px * px + py * py;
  double r = sqrt(r2);

  // check division by zero
  if (fabs(r2) < 0.0001) {
    cout << "Calculation Jacobian Error: division by Zero" << endl;
    return H_;
  }

  H_ << px / r, py / r, 0, 0,
          - py / r2, px / r2, 0, 0,
          py * (vx * py - vy * px) / (r * r2), px * (vy * px - vx * py) / (r * r2), px / r, py / r;

  return H_;
}
