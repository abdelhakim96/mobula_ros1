// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "../examples/test_functions.hpp"
#include "cg.h"
#include "gp.h"
#include "gp_utils.h"
#include "rprop.h"

#include <fstream>
#include <iomanip>
#include <iostream>

#include <Eigen/Dense>

using namespace libgp;
using namespace std;

int main(int argc, char const *argv[]) {

  string path_dir = "../../examples/data/";
  string filename_train = "WS_example_train.txt",
         filename_test = "WS_example_test.txt";
  ofstream print_results_train, print_results_test;
  print_results_train.open(path_dir + filename_train);
  //  print_results_train.open("~/cpp_codes/libgp/examples/data/WS_example_train.txt");
  if (!print_results_train) {
    cout << "Unable to open WS_example_train";
    exit(1); // terminate with error
  }
  print_results_test.open(path_dir + filename_test);
  if (!print_results_test) {
    cout << "Unable to open WS_example_test";
    exit(1); // terminate with error
  }

  const size_t kNumTrainingPoints = 100;
  const size_t kNumTestPoints = 100;
  const double kMaxRmsError = 0.01;
  const double kNoiseVariance = 1e-3;
  const double kLength = 1.0;

  //  // Random number generator.
  //  std::random_device rd;
  //  std::default_random_engine rng(rd());
  //  std::uniform_real_distribution<double> unif(0.0, 1.0);

  //  int n=4000, m=1000;
  double points_train[kNumTrainingPoints], points_test[kNumTestPoints];
  double tss = 0, error, f[kNumTestPoints], targets_train[kNumTrainingPoints],
         targets_test[kNumTestPoints];
  double var[kNumTestPoints];
  // initialize Gaussian process for 2-D input using the squared exponential
  // covariance function with additive white noise.
  GaussianProcess gp(1, "CovSum ( CovSEiso, CovNoise)");
  // initialize hyper parameter vector
  Eigen::VectorXd params(gp.covf().get_param_dim());
  params << 0.0, 0.0, -2.0;
  // set parameters of covariance function
  gp.covf().set_loghyper(params);
  // add training patterns
  for (int i = 0; i < kNumTrainingPoints; ++i) {
    double points[] = {drand48() * 4 - 2};
    //    targets = Utils::randn();
    targets_train[i] = BumpyParabola(points[0]);
    gp.add_pattern(points, targets_train[i]);
    points_train[i] = points[0];
    print_results_train << std::fixed << std::setprecision(6) << points_train[i]
                        << ", " << targets_train[i] << endl;
    //    cout<<"points_train("<<i<<") = "<<points_train[i]<<",
    //    targets_train("<<i<<") = "<<targets_train[i]<<"\n";
  }
  print_results_train.close();
  cout << "params = " << gp.covf().get_loghyper() << "\n";
  libgp::CG cg;
  cg.maximize(&gp, 100, 0);
  cout << "param_CG = " << gp.covf().get_loghyper() << "\n";

  params << 0.0, 0.0, -2.0;
  gp.covf().set_loghyper(params);
  libgp::RProp rprop;
  rprop.maximize(&gp, 100, 0);
  cout << "param_RProp = " << gp.covf().get_loghyper() << "\n";

  // total squared error
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {drand48() * 4 - 2};
    //    targets = Utils::randn();
    targets_test[i] = BumpyParabola(points[0]);
    f[i] = gp.f(points);
    var[i] = gp.var(points);
    error = f[i] - targets_test[i];
    tss += error * error;
    points_test[i] = points[0];
    //    cout<<"points_test("<<i<<") = "<<points_test[i]<<",
    //    targets_test("<<i<<") = "<<targets_test[i]
    //        <<", f("<<i<<") = "<<f[i]<<", var("<<i<<") = "<<var[i]<<",
    //        error("<<i<<") = "<<error<<"\n";
    print_results_test << std::fixed << std::setprecision(6) << points_test[i]
                       << ", " << targets_test[i] << ", " << f[i] << ", "
                       << var[i] << ", " << error << endl;
  }
  print_results_test.close();
  cout << "mse = " << tss / kNumTestPoints << endl;
  return EXIT_SUCCESS;
}
