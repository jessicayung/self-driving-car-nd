#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in
                        // ,MatrixXd &R_laser_in, MatrixXd &R_radar_in, MatrixXd &H_laser_in,
                        // MatrixXd &Hj_in
)
{
    cout << "KalmanFilter::Init" << endl;
  x_ = x_in;
  P_ = P_in; // object covariance matrix
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
/*
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  H_laser_ = H_laser_in;
  Hj_ = Hj_in;
*/
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    cout << "KalmanFilter::Predict()" << endl;
    cout << "x_: " << x_ << endl;
    cout << "F_: " << F_ << endl;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    cout << "P_: " << P_ << endl;
    cout << "Q_: " << Q_ << endl;
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    
    cout << "KalmanFilter::Update()" << endl;
    cout << "x_: " << x_ << endl;
    cout << "H_laser_: " << H_laser_ << endl;
    VectorXd z_pred = H_laser_ * x_;
    cout << "z_pred: " << z_pred << endl;
    
    cout << "z: " << z << endl;
    VectorXd y = z - z_pred;
    cout << "y: " << y << endl;
    MatrixXd Ht = H_laser_.transpose();
    cout << "Ht: " << Ht << endl;
    
    cout << "H_laser_: " << H_laser_ << endl;
    cout << "P: " << P_ << endl;
    // Error: 4x4 + 2x4
    cout << "R_laser_: " << R_laser_ << endl;
    //
    MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
    cout << "S: " << S << endl;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    cout << "K: " << K << endl;
    
    //new estimate
    cout << "y: " << y << endl;
    cout << "x_: " << x_ << endl;
    x_ = x_ + (K * y);
    cout << "x_: " << x_ << endl;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    cout << "P_: " << P_ << endl;
    cout << "I: " << I << endl;
    cout << "K: " << K << endl;
    cout << "H_laser_: " << H_laser_ << endl;
    P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    cout << "KalmanFilter::UpdateEKF()" << endl;
    // VectorXd h_ (x) = h(mu) + Jacobian * (x - mu)
    cout << "H_: " << H_ << endl;
    VectorXd z_pred = H_ * x_;
    
    VectorXd y = z - z_pred;
    cout << "y: " << y << endl;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    cout << "K_: " << K << endl;
    
    //new estimate
    
    
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    cout << "I: " << I << endl;
    P_ = (I - K * H_) * P_;
}
