// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "gp.h"
#include "gp_utils.h"
#include "gp_asparse.h"

#include <Eigen/Dense>

using namespace libgp;

int main (int argc, char const *argv[])
{
  int n=1000, m=2;
  double tss = 0, error, f, y;
  double tss1 = 0, error1, f1, y1;
  // initialize Gaussian process for 2-D input using the squared exponential 
  // covariance function with additive white noise.
  GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");

  // initialize hyper parameter vector
  Eigen::VectorXd params(gp.covf().get_param_dim());
  params << 0.0, 0.0, -2.0;
  // set parameters of covariance function
  gp.covf().set_loghyper(params);





  // Add training patterns for both GPs
  for (int i = 0; i < n; ++i) {
    double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
    y = Utils::hill(x[0], x[1]) ;
    gp.add_pattern(x, y);
   // adaptive_gp.add_pattern(x, y);
  }
   
   AdaptiveSparseGaussianProcess adaptive_gp(gp.get_sampleset_size(), gp);
  // Original GP - total squared error
  for (int i = 0; i < m; ++i) {
    double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
    
    double lamb=1;
    f = gp.f(x, lamb);
    y = Utils::hill(x[0], x[1]);
    error = f - y;
    tss += error * error;
  }

  std::cout << "mse (Original GP) = " << tss / m << std::endl;

  // Reset variables for Adaptive Sparse GP
  tss = 0;
   double  lamb = 0.98;
  // Adaptive Sparse GP - total squared error
  for (int i = 0; i < m; ++i) {
    double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
    f = adaptive_gp.f(x, lamb);
    y = Utils::hill(x[0], x[1]);
    error = f - y;
    tss += error * error;
  }

  std::cout << "mse (Adaptive Sparse GP) = " << tss / m << std::endl;

  return EXIT_SUCCESS;
}