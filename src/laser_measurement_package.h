#ifndef LASER_MEASUREMENT_PACKAGE_H_
#define LASER_MEASUREMENT_PACKAGE_H_

#include "measurement_package.h"
#include "Eigen/Dense"

class LaserMeasurementPackage : public MeasurementPackage {
public:

  /**
  * Constructor.
  */
  LaserMeasurementPackage(std::string);

  // Measurement covariance matrix (R)
  static Eigen::MatrixXd R_;

  // Measurement matrix (H)
  static Eigen::MatrixXd H_;

  // Get measurement
  Eigen::VectorXd getMeasurement();

  // Get measurement matrix (H)
  Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&);

  // Get measurement covariance matrix (R)
  Eigen::MatrixXd getMeasurementCovariance();

  // Get error (y = z - Hx')
  Eigen::VectorXd getError(const Eigen::VectorXd&);

};

#endif /* LASER_MEASUREMENT_PACKAGE_H_ */
