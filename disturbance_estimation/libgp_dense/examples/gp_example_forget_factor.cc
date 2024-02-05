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

int main(int argc, char const* argv[])
{
    const int kNumTrainingPoints = 50;
    const int kNumTestPoints = 1;

    double tss = 0, error, f, f_lambda, targets, var;
    double mse_GP = 0, mse_GP_lambda = 0;
    // initialize Gaussian process for 2-D input using the squared exponential
    // covariance function with additive white noise.
    GaussianProcess gp(2, "CovSum ( CovSEard, CovNoise)");
    // initialize hyper parameter vector
    Eigen::VectorXd params(gp.covf().get_param_dim());
    params << 0.0, 0.0, 0.0, -2.0;
    // set parameters of covariance function
    gp.covf().set_loghyper(params);
    // add training patterns
    for (int i = 0; i < kNumTrainingPoints; ++i)
    {
        double points[] = {drand48() * 4 - 2 + 3, drand48() * 4 - 2 + 3};
        targets = BumpyParabola(points[0]);
        gp.add_pattern(points, targets);
        //    cout<<"points("<<i<<") = ["<<points[0]<<","<<points[1]<<"],
        //    targets("<<i<<") = "<<targets<<"\n";
    }

    double mean_targets = 0, std_dev_targets = 1;
    Eigen::VectorXd mean_inputs(gp.get_input_dim()), std_dev_inputs(gp.get_input_dim());
    mean_inputs.setZero();
    std_dev_inputs.setOnes();

    gp.standardize_sampleset(mean_inputs, std_dev_inputs, mean_targets, std_dev_targets);
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

    double points_mat[kNumTestPoints][2];
    //  std::cout<<"test_points = ";
    for (int i = 0; i < kNumTestPoints; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            points_mat[i][j] = drand48() * 4 - 2 + 3;
            //      points_mat[i][j] = gp.get_sampleset()->x(i)(j);
            //      std::cout<<points_mat[i][j]<<", ";
        }
        //    std::cout<<"\n";
    }
    double points_mat_normalized[kNumTestPoints][2];
    //  std::cout<<"test_points_normalized = ";
    for (int i = 0; i < kNumTestPoints; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            points_mat_normalized[i][j] = (points_mat[i][j] - mean_inputs(j)) / std_dev_inputs(j);
            //      std::cout<<points_mat_normalized[i][j]<<", ";
        }  //    std::cout<<"\n";
    }

    // total squared error
    for (int i = 0; i < kNumTestPoints; ++i)
    {
        double points[] = {points_mat_normalized[i][0], points_mat_normalized[i][1]};
        f = gp.f(points) * std_dev_targets + mean_targets;
        var = gp.var(points);
        std::cout << "f = " << f << "\n";
        //    std::cout<<"var = "<<var<<"\n";
        targets = BumpyParabola(points_mat[i][0]);
        error = f - targets;
        tss += error * error;
        //    cout<<"points("<<i<<") =
        //    ["<<points_mat[i][0]<<","<<points_mat[i][1]<<"], targets("<<i<<") =
        //    "<<targets<<", f("<<i<<") = "<<f<<"\n";
    }
    mse_GP = tss / kNumTestPoints;
    cout << "mse GP = " << mse_GP << endl;

    // Testing forgetting factor-based GP
    double lambda = 0.99;
    tss = 0;
    for (int i = 0; i < kNumTestPoints; ++i)
    {
        double points[] = {points_mat_normalized[i][0], points_mat_normalized[i][1]};
        f_lambda = gp.f(points, lambda) * std_dev_targets + mean_targets;
        var = gp.var(points);
        std::cout << "f_lambda = " << f_lambda << "\n";
        //    std::cout<<"var = "<<var<<"\n";
        targets = BumpyParabola(points_mat[i][0]);
        error = f_lambda - targets;
        tss += error * error;
        //    cout<<"points("<<i<<") =
        //    ["<<points_mat[i][0]<<","<<points_mat[i][1]<<"], targets("<<i<<") =
        //    "<<targets<<", f_lambda("<<i<<") = "<<f_lambda<<"\n";
    }
    mse_GP_lambda = tss / kNumTestPoints;
    cout << "mse GP_lambda = " << mse_GP_lambda << endl;
    cout << "abs diff GP and GP_lambda = " << (double)std::abs(mse_GP - mse_GP_lambda) << endl;

    return EXIT_SUCCESS;
}
