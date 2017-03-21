#include "kalman_filter.h"
#include "measurement_package.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Process noise
  noise_ax_ = 9;
  noise_ay_ = 9;

  // State transition matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;

  // State covariance matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  // Note: The following matrices are owned by the measurement package since they differ between measurement type.
  // Measurement covariance matrix (R) 
  // Measurement matrix (H)
}

KalmanFilter::~KalmanFilter() {}

/**
 * Process new measurement
 *
 * @param measurement_pack Current measurement
 * @return The state estimate after measurement update is applied
 */
VectorXd KalmanFilter::ProcessMeasurement(MeasurementPackage *measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {
    // first measurement
    x_ = measurement_pack->getMeasurement();
    previous_timestamp_ = measurement_pack->timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return x_;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // delta_T - expressed in seconds
  double delta_T = (measurement_pack->timestamp_ - previous_timestamp_) / 1000000.0;
  if (delta_T == 0) {
    delta_T = 1;
  }
  previous_timestamp_ = measurement_pack->timestamp_;

  Predict(delta_T);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  Update(measurement_pack);

  return x_;
}

/**
 * Predicts the state and the state covariance using the process model
 *
 * @param delta_T Elapsed time since last measurement
 */
void KalmanFilter::Predict(double delta_T) {

  //1. Modify the F matrix so that time is integrated
  F_(0, 2) = delta_T;
  F_(1, 3) = delta_T;

  //2. Set the process covariance matrix Q
  
  // pre-compute a set of terms to avoid repeated calculation
  double delta_T_2 = delta_T * delta_T;
  double delta_T_3 = delta_T_2 * delta_T;
  double delta_T_4 = delta_T_3 * delta_T;

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

/**
 * Updates the state by using standard Kalman Filter equations
 *
 * @param measurement_pack The measurement at k+1
 */
void KalmanFilter::Update(MeasurementPackage *measurement_pack) {

  // error, measurement matrix, and measurement covariance matrix depend on measurement package type.
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
