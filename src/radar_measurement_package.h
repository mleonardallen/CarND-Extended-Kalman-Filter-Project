#ifndef RADAR_MEASUREMENT_PACKAGE_H_
#define RADAR_MEASUREMENT_PACKAGE_H_

#include "measurement_package.h"
#include "Eigen/Dense"

class RadarMeasurementPackage : public MeasurementPackage {
public:

  /**
  * Constructor.
  */
  RadarMeasurementPackage(std::string);

  Eigen::VectorXd getState();
  Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&);
  Eigen::MatrixXd getMeasurementCovariance();
  Eigen::VectorXd getError(const Eigen::VectorXd&);
  Eigen::VectorXd toPolar(const Eigen::VectorXd&);
  Eigen::VectorXd toCartesian(const Eigen::VectorXd&);
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd&);

  static Eigen::MatrixXd R_;

};

#endif /* RADAR_MEASUREMENT_PACKAGE_H_ */
