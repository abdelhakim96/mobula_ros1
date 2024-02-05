// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "../examples/test_functions.hpp"
#include "cg.h"
#include "gp.h"
#include "gp_sparse.h"
#include "gp_utils.h"

#include <Eigen/Dense>

using namespace libgp;
using namespace std;

int main(int argc, char const *argv[]) {
  const int kNumTrainingPoints = 5;
  const int kNumTestPoints = 1;
  //  const double kMaxRmsError = 0.01;
  //  const double kNoiseVariance = 1e-3;
  //  const double kLength = 1.0;

  //  // Random number generator.
  //  std::random_device rd;
  //  std::default_random_engine rng(rd());
  //  std::uniform_real_distribution<double> unif(0.0, 1.0);

  //  int n=4000, m=1000;
  double tss = 0, error, f, targets, var;
  // initialize Gaussian process for 2-D input using the squared exponential
  // covariance function with additive white noise.
  GaussianProcess gp(2, "CovSum ( CovSEard, CovNoise)");
  // initialize hyper parameter vector
  Eigen::VectorXd params(gp.covf().get_param_dim());
  params << 0.0, 0.0, 0.0, -2.0;
  //  params << 1.03586, 4.55848, 2.22246, -2.66138;
  // set parameters of covariance function
  gp.covf().set_loghyper(params);
  double training_points_mat[kNumTrainingPoints][2];
  double training_targets_vec[kNumTrainingPoints];
  // add training patterns
  for (int i = 0; i < kNumTrainingPoints; ++i) {
    double points[] = {drand48() * 4 - 2 + 3, drand48() * 4 - 2 + 3};
    //    targets = Utils::randn();
    targets = BumpyParabola(points[0]);
    gp.add_pattern(points, targets);
    //    cout<<"points("<<i<<") = ["<<points[0]<<","<<points[1]<<"],
    //    targets("<<i<<") = "<<targets<<"\n";
    for (int j = 0; j < 2; ++j)
      training_points_mat[i][j] = points[j];
    training_targets_vec[i] = targets;
  }
  //  for(int i = 0; i < kNumTrainingPoints; ++i) {
  //    std::cout<<"training_points_mat("<<i<<") =
  //    ["<<training_points_mat[i][0]<<","<<training_points_mat[i][1]<<"],
  //    training_targets_vec("<<i<<") = "<<training_targets_vec[i]<<"\n";
  //  }

  double mean_targets = 0, std_dev_targets = 1;
  Eigen::VectorXd mean_inputs(gp.get_input_dim()),
      std_dev_inputs(gp.get_input_dim());
  mean_inputs.setZero();
  std_dev_inputs.setOnes();

  //  gp.standardize_sampleset(mean_targets, std_dev_targets);
  //  gp.standardize_sampleset(mean_inputs, std_dev_inputs);
  gp.standardize_sampleset(mean_inputs, std_dev_inputs, mean_targets,
                           std_dev_targets);
  //  std::cout<<"mean_targets = ["<<mean_targets<<"]\n";
  //  std::cout<<"std_dev_targets = ["<<std_dev_targets<<"]\n";
  //  std::cout<<"mean_inputs = ["<<mean_inputs(0)<<","<<mean_inputs(1)<<"]\n";
  //  std::cout<<"std_dev_inputs =
  //  ["<<std_dev_inputs(0)<<","<<std_dev_inputs(1)<<"]\n";

  //  for(int i = 0; i < kNumTrainingPoints; ++i)
  //  {
  //    cout<<"points("<<i<<") =
  //    ["<<gp.get_sampleset()->x(i)(0)<<","<<gp.get_sampleset()->x(i)(1)<<"],
  //    targets("<<i<<") = "<<gp.get_sampleset()->y(i)<<"\n";
  //  }

  libgp::CG cg;
  cg.maximize(&gp, 100, 0);
  params = gp.covf().get_loghyper();
  cout << "param_CG = " << params << "\n";

  //  // Create Sparse GP
  SparseGaussianProcess sgp(gp.get_sampleset_size(), gp);
  sgp.sparsify();
  std::cout << "sgp sample size = " << sgp.get_sampleset_size() << "\n";
  //  for(int i = 4; i < 5; ++i) {
  //    double x[] = {(training_points_mat[i][0] -
  //    mean_inputs(0))/std_dev_inputs(0),
  //                  (training_points_mat[i][1] -
  //                  mean_inputs(1))/std_dev_inputs(1)};
  //    double y = (training_targets_vec[i] - mean_targets)/std_dev_targets;
  //    sgp.add_pattern(x, y);
  //  }

  //  // Check if it is important to maximize the loghyper. If we change the
  //  loghyper, all the matrices will change! cg.maximize(&sgp, 100, 0); params
  //  = sgp.covf().get_loghyper(); cout<<"param1_CG = "<<params<<"\n";

  //  string filename = "../../examples/data/gp/gp.dat";
  //  gp.write(const_cast<char*>(filename.c_str()));

  //  GaussianProcess gp1(const_cast<char*>(filename.c_str()));
  //  std::cout<<"gp1 mean_targets = ["<<gp1.get_sampleset()->mean_y()<<"]\n";
  //  std::cout<<"gp1 std_dev_targets =
  //  ["<<gp1.get_sampleset()->std_dev_y()<<"]\n"; std::cout<<"gp1 mean_inputs =
  //  ["<<gp1.get_sampleset()->mean_x(0)<<","<<gp1.get_sampleset()->mean_x(1)<<"]\n";
  //  std::cout<<"gp1 std_dev_inputs =
  //  ["<<gp1.get_sampleset()->std_dev_x(0)<<","<<gp1.get_sampleset()->std_dev_x(1)<<"]\n";

  double points_mat[kNumTestPoints][2];
  //  std::cout<<"test_points = ";
  for (int i = 0; i < kNumTestPoints; ++i) {
    for (int j = 0; j < 2; ++j) {
      points_mat[i][j] = drand48() * 4 - 2 + 3;
      //      points_mat[i][j] = gp.get_sampleset()->x(i)(j);
      //      std::cout<<points_mat[i][j]<<", ";
    }
    //    std::cout<<"\n";
  }
  double points_mat_normalized[kNumTestPoints][2];
  //  std::cout<<"test_points_normalized = ";
  for (int i = 0; i < kNumTestPoints; ++i) {
    for (int j = 0; j < 2; ++j) {
      points_mat_normalized[i][j] =
          (points_mat[i][j] - mean_inputs(j)) / std_dev_inputs(j);
      //      std::cout<<points_mat_normalized[i][j]<<", ";
    }
    //    std::cout<<"\n";
  }

  // total squared error
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {points_mat_normalized[i][0],
                       points_mat_normalized[i][1]};
    f = gp.f(points) * std_dev_targets + mean_targets;
    var = gp.var(points);
    //    std::tie(f, var) = gp.f_var(points);
    //    f = f*std_dev_targets + mean_targets;
    //    targets = Utils::randn();
    std::cout << "f = " << f << "\n";
    //    std::cout<<"var = "<<var<<"\n";
    targets = BumpyParabola(points_mat[i][0]);
    error = f - targets;
    tss += error * error;
    //    cout<<"points("<<i<<") =
    //    ["<<points_mat[i][0]<<","<<points_mat[i][1]<<"], targets("<<i<<") =
    //    "<<targets<<", f("<<i<<") = "<<f<<"\n";
  }
  cout << "mse GP = " << tss / kNumTestPoints << endl;
  // remove one test point
  gp.remove_sample(2);
  Eigen::VectorXd sx2;
  sx2.setConstant(2, 0.0);
  std::cout << "GP hyperpar_needs_update = " << gp.get_hyperpar_needs_update()
            << "\n";
  tss = 0;
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {points_mat_normalized[i][0],
                       points_mat_normalized[i][1]};
    f = gp.f_noisy_inp(points, sx2) * std_dev_targets + mean_targets;
    var = gp.var_noisy_inp(points, gp.f_noisy_inp(points, sx2), sx2);
    std::cout << "f_noisy_inp = " << f << "\n";
    //    std::cout<<"var_noisy_inp = "<<var<<"\n";
    //    if (var < 0)
    //      std::cout<<"var_noisy_inp = "<<var<<"\n";
    targets = BumpyParabola(points_mat[i][0]);
    error = f - targets;
    tss += error * error;
  }
  std::cout << "GP hyperpar_needs_update = " << gp.get_hyperpar_needs_update()
            << "\n";
  cout << "mse GP noisy = " << tss / kNumTestPoints << endl;

  std::cout << "sgp sample size = " << sgp.get_sampleset_size() << "\n";
  tss = 0;
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {points_mat_normalized[i][0],
                       points_mat_normalized[i][1]};
    f = sgp.f(points) * std_dev_targets + mean_targets;
    var = sgp.var(points);
    //    std::tie(f, var) = sgp.f_var(points);
    //    f = f*std_dev_targets + mean_targets;
    std::cout << "f = " << f << "\n";
    //    std::cout<<"var = "<<var<<"\n";
    targets = BumpyParabola(points_mat[i][0]);
    error = f - targets;
    tss += error * error;
  }
  cout << "mse SGP = " << tss / kNumTestPoints << endl;
  // remove one test point
  sgp.sparsify(2);
  std::cout << "SGP hyperpar_needs_update = " << sgp.get_hyperpar_needs_update()
            << "\n";
  tss = 0;
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {points_mat_normalized[i][0],
                       points_mat_normalized[i][1]};
    f = sgp.f_noisy_inp(points, sx2) * std_dev_targets + mean_targets;
    var = sgp.var_noisy_inp(points, sgp.f_noisy_inp(points, sx2), sx2);
    std::cout << "f_noisy_inp = " << f << "\n";
    //    std::cout<<"var_noisy_inp = "<<var<<"\n";
    //    if (var < 0)
    //      std::cout<<"var_noisy_inp = "<<var<<"\n";
    targets = BumpyParabola(points_mat[i][0]);
    error = f - targets;
    tss += error * error;
  }
  std::cout << "SGP hyperpar_needs_update = " << sgp.get_hyperpar_needs_update()
            << "\n";
  cout << "mse SGP noisy = " << tss / kNumTestPoints << endl;

  //  for (size_t i=0; i<gp.get_sampleset_size(); ++i)
  //  {
  //    std::cout<<"before gp.sampleset.x = "<<gp.get_sampleset()->x(i)<<"\n";
  //    std::cout<<"before gp.sampleset.y = "<<gp.get_sampleset()->y(i)<<"\n";
  //  }
  gp.set_sampleset(*sgp.get_sampleset());
  //  for (size_t i=0; i<gp.get_sampleset_size(); ++i)
  //  {
  //    std::cout<<"after gp.sampleset.x = "<<gp.get_sampleset()->x(i)<<"\n";
  //    std::cout<<"after gp.sampleset.y = "<<gp.get_sampleset()->y(i)<<"\n";
  //  }
  tss = 0;
  // total squared error
  for (int i = 0; i < kNumTestPoints; ++i) {
    double points[] = {points_mat_normalized[i][0],
                       points_mat_normalized[i][1]};
    f = gp.f(points) * std_dev_targets + mean_targets;
    var = gp.var(points);
    //    std::tie(f, var) = gp.f_var(points);
    //    f = f*std_dev_targets + mean_targets;
    //    targets = Utils::randn();
    std::cout << "f = " << f << "\n";
    //    std::cout<<"var = "<<var<<"\n";
    targets = BumpyParabola(points_mat[i][0]);
    error = f - targets;
    tss += error * error;
    //    cout<<"points("<<i<<") =
    //    ["<<points_mat[i][0]<<","<<points_mat[i][1]<<"], targets("<<i<<") =
    //    "<<targets<<", f("<<i<<") = "<<f<<"\n";
  }
  cout << "mse newGP = " << tss / kNumTestPoints << endl;

  return EXIT_SUCCESS;
}
