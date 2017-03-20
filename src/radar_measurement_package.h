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

  // Measurement Covariance Matrix
  static Eigen::MatrixXd R_;

  // Get Measurement (Cartesian)
  Eigen::VectorXd getMeasurement();

  // Get Measurement Matrix (H)
  Eigen::MatrixXd getMeasurementMatrix(const Eigen::VectorXd&);

  // Get Measurement Covariance Matrix (R)
  Eigen::MatrixXd getMeasurementCovariance();

  // Get Error (y = z - Hx')
  Eigen::VectorXd getError(const Eigen::VectorXd&);

  // Convert Measurement from Cartesian to Polar
  Eigen::VectorXd toPolar(const Eigen::VectorXd&);

  // Convert Measurement from Polar to Cartesian
  Eigen::VectorXd toCartesian(const Eigen::VectorXd&);

  // Calculate Jacobian
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd&);

};

#endif /* RADAR_MEASUREMENT_PACKAGE_H_ */
