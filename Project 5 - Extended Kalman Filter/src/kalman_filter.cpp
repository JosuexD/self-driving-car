#include "kalman_filter.h"
#include <iostream>

using namespace std;
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

// Below is the predict formula for the kalman filter. it follows the algorithm line by line from the reference ekf pdf.
// The only slight difference to note is the fact that we don't have a 'u' added to the initial prediction, this is due
// to the fact that v represents 'process noise. Since we don't have process noise we lave it out
void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd F_t = F_.transpose();
    P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    // lines 13-15 of the Kalman Filter Equations
    VectorXd z_pred = H_ * x_; // Prediceted state. line 3 of the Kalman Filter Equations in C++
    VectorXd y = z - z_pred;
    MatrixXd  H_t = H_.transpose();
    MatrixXd S = H_ * P_ * H_t + R_;
    MatrixXd S_inverse = S.inverse();
    MatrixXd K = P_ * H_t * S_inverse;

    // New estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size); // Identity matrix of size of our calculated prediction
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Recover explicitly status information
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // Map predicted state into measurement space
    double rho     = sqrt(px * px + py * py);
    double phi	   = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / std::max(rho, 0.0001);

    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;

    VectorXd y = z - z_pred;

    // Normalize angle between
    while (y(1) > M_PI) y(1) -= 2 * M_PI;
    while (y(1) < -M_PI) y(1) += 2 * M_PI;

    // Updating the routine
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();

    // Compute Kalman gain
    MatrixXd K = P_ * Ht * Si;

    // Update estimate value
    x_ = x_ + K * y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
