#include "kalman_filter.h"
#include "measurement_package.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  noise_ax_ = 9;

  noise_ay_ = 9;

  // State Transition Matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  // State Covariance Matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  // note: Measurement Covariance Matrix (R_) found in measurement packages.
}

KalmanFilter::~KalmanFilter() {}

VectorXd KalmanFilter::ProcessMeasurement(MeasurementPackage *measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    // first measurement
    x_ = measurement_pack->getState();
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return x_;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // delta_T - expressed in seconds
  float delta_T = (measurement_pack->timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack->timestamp_;

  Predict(delta_T);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  Update(measurement_pack);

  return x_;
}

void KalmanFilter::Predict(float delta_T) {

  //1. Modify the F matrix so that time is integrated
  F_(0, 2) = delta_T;
  F_(1, 3) = delta_T;

  //2. Set the process covariance matrix Q
  
  // pre-compute a set of terms to avoid repeated calculation
  float delta_T_2 = delta_T * delta_T;
  float delta_T_3 = delta_T_2 * delta_T;
  float delta_T_4 = delta_T_3 * delta_T;
  
  Q_ = MatrixXd(4, 4);
  Q_ << delta_T_4/4 * noise_ax_, 0, delta_T_3/2 * noise_ax_, 0,
    0, delta_T_4/4 * noise_ay_, 0, delta_T_3/2 * noise_ay_,
    delta_T_3/2 * noise_ax_, 0, delta_T_2 * noise_ax_, 0,
    0, delta_T_3/2 * noise_ay_, 0, delta_T_2 * noise_ay_;

  //3. Predict
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(MeasurementPackage *measurement_pack) {

  // error, measurement matrix, and measurement covariance depend on measurement package type.
  VectorXd y = measurement_pack->getError(x_);
  MatrixXd H = measurement_pack->getMeasurementMatrix(x_);
  MatrixXd R = measurement_pack->getMeasurementCovariance();

  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;

  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}
