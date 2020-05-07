#include "kalman_filter.h"
#include <iostream>

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
  
  VectorXd z_predict = H_ * x_;
  VectorXd y = z - z_predict;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  /**
   *  Radar
   * 
   *  nonlinear measurement function.
   *  convert from cartesian coordinates [px, py, vx, vy] to 
   *  polar [rho, phi, rho_dot].
   */
  
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float den = px*px + py*py;
  if (fabs(den) < 1e-6) {
    den = 1e-6;
  }
  float rho = pow(den, 1.0/2.0);
  float phi = atan2(py,px);

  float rho_dot = (px*vx+py*vy)/rho;
  VectorXd z_predict = VectorXd(3);
  z_predict << rho, phi, rho_dot;
  VectorXd y = z - z_predict;

  // make sure (phi - phi_predict) is within [-pi, pi]
  // example: phi = 3.12737 rad (179.1851 deg)
  //          phi_predict = -3.12737 rad (-179.1851 deg)
  //          difference = 6.25474 rad (358.370204 deg)
  //
  //          THAT IS A HUGH DIFFERENCE!!!
  //          this caused a dump in xy and vy
  //          so wrap the diff angle.

  while (y[1] > M_PI || y[1] < -M_PI) {
    if (y[1] > M_PI) {
      y[1] -= 2.0*M_PI;
    }
    else {
      y[1] += 2.0*M_PI;
    }
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
