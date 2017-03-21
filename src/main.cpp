#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"

#include "tools.h"
#include "kalman_filter.h"
#include "measurement_package.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  // Create a Fusion Extended Kalman Filter instance
  KalmanFilter kf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  string line;
  MeasurementPackage *meas_package;

  //Call the Extended Kalman Filter based fusion
  while (getline(in_file_, line)) {

    meas_package = MeasurementPackage::create(line);

    // start filtering from the second frame (the speed is unknown in the first frame)
    VectorXd x_ = kf.ProcessMeasurement(meas_package);

    // output the estimation
    out_file_ << x_(0) << "\t";
    out_file_ << x_(1) << "\t";
    out_file_ << x_(2) << "\t";
    out_file_ << x_(3) << "\t";

    // output the measurements
    VectorXd state = meas_package->getMeasurement();
    out_file_ << state[0] << "\t";
    out_file_ << state[1] << "\t";

    // // output the ground truth packages
    out_file_ << meas_package->gt_values_(0) << "\t";
    out_file_ << meas_package->gt_values_(1) << "\t";
    out_file_ << meas_package->gt_values_(2) << "\t";
    out_file_ << meas_package->gt_values_(3) << "\n";

    estimations.push_back(x_);
    ground_truth.push_back(meas_package->gt_values_);

  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
