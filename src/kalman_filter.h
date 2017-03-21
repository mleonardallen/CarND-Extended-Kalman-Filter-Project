#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {
public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Predicts the state and the state covariance using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(double);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param measurement_pack The measurement at k+1
   */
  void Update(MeasurementPackage *measurement_pack);

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  Eigen::VectorXd ProcessMeasurement(MeasurementPackage *measurement_pack);


private:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // process noise
  double noise_ax_;

  // process noise
  double noise_ay_;

};

#endif /* KALMAN_FILTER_H_ */
