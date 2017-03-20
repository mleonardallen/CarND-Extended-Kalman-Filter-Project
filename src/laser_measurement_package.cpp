#include "Eigen/Dense"
#include <sstream>
#include <iostream>

#include "laser_measurement_package.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * Measurement Covariance
 */
MatrixXd LaserMeasurementPackage::R_ = (
  MatrixXd(2, 2) << 0.0225, 0,
    0, 0.0225
).finished();

/**
 * Measurement Matrix
 */
MatrixXd LaserMeasurementPackage::H_ = (
  MatrixXd(2, 4) << 1, 0, 0, 0,
    0, 1, 0, 0
).finished();

/*
 * Constructor.
 * @param line sensor data
 */
LaserMeasurementPackage::LaserMeasurementPackage(string line) {

  istringstream iss(line);
  sensor_type_ = MeasurementPackage::LASER;

  // Raw Measurement
  raw_measurements_ = VectorXd(2);
  float x;
  float y;
  string sensor_type;

  iss >> sensor_type >> x >> y >> timestamp_;
  raw_measurements_ << x, y;

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
 */
VectorXd LaserMeasurementPackage::getState() {

  VectorXd init = VectorXd(4);
  init << raw_measurements_[0], raw_measurements_[1], 0, 0;

  return init;
}

/*
 * Get measurement matrix
 * @param x_state the predicted state
 */
MatrixXd LaserMeasurementPackage::getMeasurementMatrix(const VectorXd& x_state) {
  return H_;
}

/**
 * Get measurement covariance
 */
MatrixXd LaserMeasurementPackage::getMeasurementCovariance() {
  return R_;
}

/**
 * Get error
 * @param x_state the predicted state
 */
Eigen::VectorXd LaserMeasurementPackage::getError(const VectorXd& x_state) {
  MatrixXd H = getMeasurementMatrix(x_state);
  VectorXd x_pred = H * x_state;

  return raw_measurements_ - x_pred;
}