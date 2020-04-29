#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    return rmse;

  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  float x = x_state(0);
  float y = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  if (fabs(x) < EPS ) x = EPS;
  if (fabs(y) < EPS ) y = EPS;
  float c1 = x * x + y * y;
  if (fabs(c1) < EPS) c1 = EPS;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);
  Hj << (x / c2), (y / c2), 0, 0, -(y / c1), (x / c1), 0, 0,
      y * (vx * y - vy * x) / c3, x * (x * vy - y * vx) / c3, x / c2,
      y / c2;
  return Hj;
}
