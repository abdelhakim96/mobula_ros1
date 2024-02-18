// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#include "gp.h"
#include "gp_utils.h"
#include "rprop.h"
#include "cg.h"
#include <iomanip>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

using namespace libgp;
using namespace std;
using namespace Eigen;

int main (int argc, char const *argv[])
{
  const size_t InputDim = 3, timeDim = 1;

  string path_dir = "../../examples/data/", filename_train = "data_train.txt", filename_test = "data_test.txt";
  ifstream inFile;
  ofstream outFile;

  std::vector<VectorXd> input_data_train, target_data_train,
                        input_data_test, target_data_test;

  inFile.open(path_dir + filename_train);
  if (!inFile) {
    cout << "Unable to open "<<filename_train<<"\n";
    exit(1); // terminate with error
  }
  double x;
  VectorXd x_vec(InputDim);
  std::vector<double> time_vec_train, time_vec_test;
  while (!inFile.eof())
  {
    for (int i=0; i<timeDim; ++i)
    {
      inFile >> x;
      time_vec_train.push_back(x);
    }
    for (int i=0; i<InputDim; ++i)
    {
      inFile >> x;
      x_vec[i] = x;
    }
    input_data_train.push_back(x_vec);
    inFile >> x;
    target_data_train.push_back(VectorXd::Constant(1, x));
  }
  inFile.close();
  input_data_train.pop_back();
  target_data_train.pop_back();

//  for (size_t ii = 0; ii < input_data_train.size(); ii++)
//    cout<<"time_vec_train("<<ii<<") = "<<time_vec_train.at(ii)<<", input_data_train("<<ii<<") = "
//        <<input_data_train.at(ii)<<", target_data_train("<<ii<<") = "<<target_data_train.at(ii)<<"\n";

  inFile.open(path_dir + filename_test);
  if (!inFile) {
    cout << "Unable to open "<<filename_test<<"\n";
    exit(1); // terminate with error
  }
  while (!inFile.eof())
  {
    for (int i=0; i<timeDim; ++i)
    {
      inFile >> x;
      time_vec_test.push_back(x);
    }
    for (int i=0; i<InputDim; ++i)
    {
      inFile >> x;
      x_vec[i] = x;
    }
    input_data_test.push_back(x_vec);
    inFile >> x;
    target_data_test.push_back(VectorXd::Constant(1, x));
  }
  inFile.close();
  input_data_test.pop_back();
  target_data_test.pop_back();

//  for (size_t ii = 0; ii < input_data_test.size(); ii++)
//    cout<<"time_vec_test("<<ii<<") = "<<time_vec_test.at(ii)<<", input_data_test("<<ii<<") = "
//        <<input_data_test.at(ii)<<", target_data_test("<<ii<<") = "<<target_data_test.at(ii)<<"\n";

  const size_t NumTrainingPoints = input_data_train.size();
  const size_t NumTestPoints = input_data_test.size();
//  const double MaxRmsError = 0.01;
  const std::vector<double> kLength_sqr_0(InputDim,1);
  const double kSignalVariance_0 = 1e-1;
  const double kNoiseVariance_0 = 1e-2;
  const double kGradUpdates = 1000;
//  const double kLength_sqr_0 = 0.3536;
//  const double kSignalVariance_0 = 0.032;
//  const double kNoiseVariance_0 = 0.0135;
  double tss = 0, error, targets, mu_test, var_test;

  // initialize Gaussian process for 1-D input using the squared exponential
  // covariance function with additive white noise.
//  GaussianProcess gp(InputDim, "CovSum ( CovSEiso, CovNoise)");
  GaussianProcess gp(InputDim, "CovSum ( CovSEard, CovNoise)");
//  GaussianProcess gp(InputDim, "CovSum ( CovMatern5iso, CovNoise)");
  // initialize hyper parameter vector
  Eigen::VectorXd params(gp.covf().get_param_dim());
  for (int i=0; i<InputDim; ++i)
    params(i) = log(kLength_sqr_0[i]);
  params(InputDim) = log(kSignalVariance_0);
  params(InputDim+1) = log(kNoiseVariance_0);

//  params << log(kLength_sqr_0[0]), log(kSignalVariance_0), log(kNoiseVariance_0);
  // set parameters of covariance function
  gp.covf().set_loghyper(params);

//  cout<<"gp.get_input_dim() = "<<gp.get_input_dim()<<", gp.get_sampleset_size() = "<<gp.get_sampleset_size()<<"\n";

  outFile.open(path_dir + "Mat_" + filename_train);
  if (!outFile) {
    cout << "Unable to open "<<"Mat_" + filename_train<<"\n";
    exit(1); // terminate with error
  }
  // add training patterns
  for(int i = 0; i < NumTrainingPoints; ++i) {
    double points[InputDim];
    for (int ii=0; ii<InputDim; ++ii)
      points[ii] = input_data_train.at(i)(ii);
    targets = target_data_train.at(i)(0);
    gp.add_pattern(points, targets);
//    cout<<"points_train("<<i<<") = "<<points[0]<<", targets_train("<<i<<") = "<<targets<<"\n";
    outFile<< std::fixed << std::setprecision(6)<< time_vec_train.at(i) <<", " ;
    for (int ii=0; ii<InputDim; ++ii)
      outFile<<points[ii]<< ", ";
    outFile<< targets<< endl;
  }
  outFile.close();

  cout<<"params_0 = ";
  for (int i=0; i<params.size(); ++i)
    cout<<exp(gp.covf().get_loghyper()(i))<<", ";
  cout<<"\n";
  libgp::CG cg;
  cg.maximize(&gp, kGradUpdates, 0);
  cout<<"params_CG = ";
  for (int i=0; i<params.size(); ++i)
    cout<<exp(gp.covf().get_loghyper()(i))<<", ";
  cout<<"\n";

//  params << log(kLength_sqr_0), log(kSignalVariance_0), log(kNoiseVariance_0);
//  gp.covf().set_loghyper(params);
//  cout<<"params_0 = ";
//  for (int i=0; i<params.size(); ++i)
//    cout<<exp(gp.covf().get_loghyper()(i))<<", ";
//  cout<<"\n";
//  libgp::RProp rprop;
//  rprop.maximize(&gp, kGradUpdates, 0);
//  cout<<"params_Rprop = ";
//  for (int i=0; i<params.size(); ++i)
//    cout<<exp(gp.covf().get_loghyper()(i))<<", ";
//  cout<<"\n";

  outFile.open(path_dir + "Mat_" + filename_test);
  if (!outFile) {
    cout << "Unable to open "<<"Mat_" + filename_test<<"\n";
    exit(1); // terminate with error
  }
  // total squared error
  for(int i = 0; i < NumTestPoints; ++i) {
    double points[InputDim];
    for (int ii=0; ii<InputDim; ++ii)
      points[ii] = input_data_test.at(i)(ii);
    targets = target_data_test.at(i)(0);
    mu_test= gp.f(points);
    var_test = gp.var(points);
    error = mu_test - targets;
    tss += error*error;
//    cout<<"points_test("<<i<<") = "<<points[0]<<", targets_test("<<i<<") = "<<targets
//        <<", f("<<i<<") = "<<mu_test<<", var("<<i<<") = "<<var_test<<", error("<<i<<") = "<< error<<"\n";
    outFile<< std::fixed << std::setprecision(6)<< time_vec_test.at(i) <<", " ;
    for (int ii=0; ii<InputDim; ++ii)
      outFile<<points[ii]<< ", ";
    outFile<< targets<< ", "<< mu_test<< ", "  << var_test<<", "<< error<< endl;
  }
  outFile.close();
  cout << "mse = " << tss/NumTestPoints << endl;
  return EXIT_SUCCESS;
}
