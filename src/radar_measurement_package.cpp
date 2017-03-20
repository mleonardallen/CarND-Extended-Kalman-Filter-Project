#include "Eigen/Dense"
#include <sstream>
#include <iostream>
#include <math.h>

#include "radar_measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * Measurement covariance
 */
MatrixXd RadarMeasurementPackage::R_ = (
  MatrixXd(3, 3) << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09
).finished();

/**
 * Constructor
 * @param line measurement data
 */
RadarMeasurementPackage::RadarMeasurementPackage(string line) {

  istringstream iss(line);

  sensor_type_ = MeasurementPackage::RADAR;
  raw_measurements_ = VectorXd(3);

  float ro;
  float theta;
  float ro_dot;

  string sensor_type;

  iss >> sensor_type >> ro >> theta >> ro_dot >> timestamp_;
  raw_measurements_ << ro, theta, ro_dot;

  // Ground Truth
  gt_values_ = VectorXd(4);
  float x_gt;
  float y_gt;
  float vx_gt;
  float vy_gt;

  iss >> x_gt >> y_gt >> vx_gt >> vy_gt;
  gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
}

/*
 * Get state from measurement.
 * note: converts measurement from polar to cartesian.
 */
VectorXd RadarMeasurementPackage::getState() {
  return toCartesian(raw_measurements_);
}

/**
 * @param x_state the predicted state
 */
MatrixXd RadarMeasurementPackage::getMeasurementMatrix(const VectorXd& x_state) {
  return CalculateJacobian(x_state);
}

/**
 * Get measurement covariance
 */
MatrixXd RadarMeasurementPackage::getMeasurementCovariance() {
  return R_;
}

/**
 * Get prediction error
 * @param x_state the predicted state
 * note: first converts predicted state to polar representation
 */
VectorXd RadarMeasurementPackage::getError(const VectorXd& x_state) {
  VectorXd x_pred = toPolar(x_state);

  return raw_measurements_ - x_pred;
}

/**
 * Convert polar to cartesian
 */
VectorXd RadarMeasurementPackage::toCartesian(const VectorXd& x_state_polar) {
  VectorXd x_state_cartesian = VectorXd(4);

  float ro = x_state_polar[0];
  float theta = x_state_polar[1];

  x_state_cartesian << ro * cos(theta), ro * sin(theta), 0, 0;

  return x_state_cartesian;
}

/**
 * Convert cartesian to polar
 */
VectorXd RadarMeasurementPackage::toPolar(const VectorXd& x_state) {

  VectorXd x_state_polar = VectorXd(3);

  //recover state parameters
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];

  // pre-compute a set of terms to avoid repeated calculation
  float px_2 = px * px;
  float py_2 = py * py;
  float sqrt_px_2_py_2 = sqrt(px_2 + py_2);

  x_state_polar[0] = sqrt_px_2_py_2;
  x_state_polar[1] = atan(py/px);
  x_state_polar[2] = (px*vx + py*vy) / sqrt_px_2_py_2;

  return x_state_polar;
}

/**
 * Calculate Jacobian
 */
MatrixXd RadarMeasurementPackage::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3,4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}