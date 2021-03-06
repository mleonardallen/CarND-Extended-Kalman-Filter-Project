#include "measurement_package.h"
#include "laser_measurement_package.h"
#include "radar_measurement_package.h"

#include "Eigen/Dense"
#include <sstream>
#include <iostream>

using namespace std;
using Eigen::VectorXd;

/**
 * Get MeasurementPackage instance
 * Note: Creates concrete class based on first element in the sensor data:
 *
 *  L: Laser
 *  R: Radar
 *
 * @param line Sensor data
 * @return Concrete MeasurementPackage instance
 */
MeasurementPackage *MeasurementPackage::create(string line) {

  MeasurementPackage *meas_package;
  istringstream iss(line);
  string sensor_type;

  iss >> sensor_type;

  if (sensor_type.compare("L") == 0) {
    // LASER MEASUREMENT
    meas_package = new LaserMeasurementPackage(line);
  } else if (sensor_type.compare("R") == 0) {
    // RADAR MEASUREMENT
    meas_package = new RadarMeasurementPackage(line);
  }

  return meas_package;
}
