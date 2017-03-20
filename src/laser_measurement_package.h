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

  Eigen::VectorXd getState();
  Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&);
  Eigen::MatrixXd getMeasurementCovariance();
  Eigen::VectorXd getError(const Eigen::VectorXd&);

  static Eigen::MatrixXd R_;
  static Eigen::MatrixXd H_;
};

#endif /* LASER_MEASUREMENT_PACKAGE_H_ */
