#include "kalman_filter.h"

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
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;                        // State update
  P_ = F_ * P_ * F_.transpose() + Q_;  // State covariance update
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;  // residue
  NormalizeAngle(y(1));      // angle normalization
  UpdateCommon(y);           // Update X and P
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double theta = atan2(x_(1), x_(0));
  double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));

  double rho_dot =
      (fabs(rho) < EPS) ? 0 : (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;
  VectorXd y = z - h;    //
  NormalizeAngle(y(1));  // angle normalization
  UpdateCommon(y);       // update X and P
}

/**
 * @brief As it turns out, we need to normalize the phase of the innovation
 * vector y(1) here to the interval [-pi,pi] in order to ensure the filter's
 * stability and accuracy. Only then will you be able to meet the required RMSE
 * values.
 *
 * @param theta Angle in radians
 */
void KalmanFilter::NormalizeAngle(double &theta) {
  theta = atan2(sin(theta), cos(theta));
}

void KalmanFilter::UpdateCommon(const Eigen::VectorXd &y) {
  const MatrixXd Ht = H_.transpose();    // intermediate
  const MatrixXd S = H_ * P_ * Ht + R_;  // innovation
  const MatrixXd Si = S.inverse();       // innovation inverse
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;  // Kalman gain
  x_ += K * y;                  // update states
  P_ -= K * H_ * P_;            // update state covariance
}