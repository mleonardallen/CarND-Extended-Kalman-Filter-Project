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

  Eigen::VectorXd raw_measurements_;
  Eigen::VectorXd gt_values_;

  static MeasurementPackage *create(std::string);

  virtual Eigen::VectorXd getState() = 0;
  virtual Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&) = 0;
  virtual Eigen::MatrixXd getMeasurementCovariance() = 0;
  virtual Eigen::VectorXd getError(const Eigen::VectorXd&) = 0;
  static Eigen::MatrixXd R_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
