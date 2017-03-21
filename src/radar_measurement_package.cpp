#include "Eigen/Dense"
#include <sstream>
#include <iostream>
#include <math.h>

#include "radar_measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * Measurement covariance matrix (R)
 */
MatrixXd RadarMeasurementPackage::R_ = (
  MatrixXd(3, 3) << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09
).finished();

/**
 * Constructor
 *
 * @param line Sensor data
 */
RadarMeasurementPackage::RadarMeasurementPackage(string line) {

  istringstream iss(line);

  sensor_type_ = MeasurementPackage::RADAR;
  raw_measurements_ = VectorXd(3);

  double ro;
  double theta;
  double ro_dot;

  string sensor_type;

  iss >> sensor_type >> ro >> theta >> ro_dot >> timestamp_;
  raw_measurements_ << ro, theta, ro_dot;

  // Ground Truth
  gt_values_ = VectorXd(4);
  double x_gt;
  double y_gt;
  double vx_gt;
  double vy_gt;

  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
}

/*
 * Note: Converts measurement from polar to cartesian
 * @return Measurement matrix (H)
 */
VectorXd RadarMeasurementPackage::getMeasurement() {
  return toCartesian(raw_measurements_);
}

/**
 * Get measurement matrix (H)
 * Note: Uses CalculateJacobian function
 *
 * @param x_state The predicted state
 * @return Measurement matrix (H)
 */
MatrixXd RadarMeasurementPackage::getMeasurementMatrix(const VectorXd& x_state) {
  return CalculateJacobian(x_state);
}

/**
 * Get measurement covariance matrix (R)
 *
 * @return Measurement covariance matrix (R)
 */
MatrixXd RadarMeasurementPackage::getMeasurementCovariance() {
  return R_;
}

/**
 * Get error [y = z - h(x')]
 * Note: First converts the predicted state to polar representation
 *
 * @param x_state The predicted state
 * @return Error
 */
VectorXd RadarMeasurementPackage::getError(const VectorXd& x_state) {
  VectorXd x_pred = toPolar(x_state);
  return raw_measurements_ - x_pred;
}

/**
 * Convert polar to cartesian
 *
 * @param x_state The state (polar)
 * @return The state (cartesian)
 */
VectorXd RadarMeasurementPackage::toCartesian(const VectorXd& x_state) {
  VectorXd x_state_cartesian = VectorXd(4);

  double ro = x_state(0);
  double theta = x_state(1);

  x_state_cartesian << ro * cos(theta), ro * sin(theta), 0, 0;

  return x_state_cartesian;
}

/**
 * Convert cartesian to polar
 *
 * @param x_state The state (cartesian)
 * @return The state (polar)
 */
VectorXd RadarMeasurementPackage::toPolar(const VectorXd& x_state) {

  VectorXd x_state_polar = VectorXd(3);

  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  // pre-compute a set of terms to avoid repeated calculation
  double px_2 = px * px;
  double py_2 = py * py;
  double sqrt_px_2_py_2 = sqrt(px_2 + py_2);

  // avoid division by zero
  if (sqrt_px_2_py_2 == 0) {
    cout << "toPolar () - Error - Division by Zero" << endl;
    x_state_polar << 0, 0, 0;
    return x_state_polar;
  }

  x_state_polar(0) = sqrt_px_2_py_2;
  x_state_polar(1) = atan2(py, px);
  x_state_polar(2) = (px*vx + py*vy) / sqrt_px_2_py_2;

  return x_state_polar;
}

/**
 * Calculate Jacobian
 * 
 * @param x_state The predicted state
 * @return Jacobian for measurement matrix (H)
 */
MatrixXd RadarMeasurementPackage::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);

  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    Hj << 0, 0, 0, 0,
      1e+9, 1e+9, 0, 0,
      0, 0, 0, 0;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}