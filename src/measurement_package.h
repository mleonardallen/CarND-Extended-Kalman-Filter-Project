#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"
#include <sstream>

class MeasurementPackage {
public:

  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // Raw Measurements
  Eigen::VectorXd raw_measurements_;

  // Ground Truth Values
  Eigen::VectorXd gt_values_;

  // Measurement Covariance Matrix
  static Eigen::MatrixXd R_;

  // MeasurementPackage Factory
  static MeasurementPackage *create(std::string);

  // Get Measurement
  virtual Eigen::VectorXd getMeasurement() = 0;

  // Get Measurement Matrix (H)
  virtual Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&) = 0;

  // Get Measurement Covariance Matrix (R)
  virtual Eigen::MatrixXd getMeasurementCovariance() = 0;

  // Get Error (y = z - Hx')
  virtual Eigen::VectorXd getError(const Eigen::VectorXd&) = 0;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
