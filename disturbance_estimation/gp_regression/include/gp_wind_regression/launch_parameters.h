/**
 * @file   launch_parameters.h
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief  Structs describing launch parameters for GP model.
 */

#ifndef _LAUNCH_PARAMETERS_H_
#define _LAUNCH_PARAMETERS_H_

/// External imports
#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace wind_regression
{
enum GPModelType
{
    CONCATENATED,
    OTHER
};

enum GPInputDataType
{
    TYPE0,
    TYPE1
};

enum FeatureDataModelType
{
    SUBSCRIBE,
    COMPUTE
};

enum FeatureDataType
{
    MODELDIFFERENCE,
    STATES,
    CONTROLS
};

enum GPDirectionType
{
    X,
    Y,
    Z
};

enum HyperOptimizeMethod
{
    CG,
    RPROP
};

struct CovarianceFunctionParams
{
    Eigen::VectorXd ell_t0_, sx_t0_;
    double sf_t0_, sn_t0_;
    std::string covfun1_, covfun2_, final_covfun_;
};

struct InputDataParams
{
    GPInputDataType gp_input_data_type_;
    FeatureDataModelType feature_data_model_type_;
    std::vector<FeatureDataType> feature_data_type_;
    size_t num_features_, data_history_, num_inputs_;
};

struct GPParams
{
    std::string filepath_gp_initial_, filename_gp_initial_, file_gp_initial_;
    bool create_new_gp_initial_, use_gp_initial_;
    GPModelType gp_model_type_;
    size_t max_initial_rec_time_sec_, num_window_points_past_, num_predict_points_future_;
    double rec_rate_factor_;
    bool use_sparse_gp_initial_;
    double sizefactor_sparse_gp_initial_;  // Max = 1
    InputDataParams input_data_params_;
    CovarianceFunctionParams covariance_function_params_;
    HyperOptimizeMethod hyper_optimize_method_;
    int max_gradient_updates_;

    double mean_t0_, variance_t0_;

    //Eigen::VectorXd inputs_noise_variance_;
};

struct RunParams
{
    std::string ros_namespace_;
    int debug_flag_;     // Flag to run in debug mode
    int ros_loop_rate_;  // Sampling rate
    GPDirectionType gp_direction_type_;
    int filter_window_;
    double mass_;
};

}  // namespace wind_regression

#endif
