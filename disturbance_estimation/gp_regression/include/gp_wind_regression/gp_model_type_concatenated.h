/**
 * @file   gp_wind_regression_main.h
 * @author Mohit Mehndiratta
 * @date   Sep 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief
 */

#ifndef GP_MODEL_TYPE_CONCATENATED_H
#define GP_MODEL_TYPE_CONCATENATED_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports
#include <Eigen/Dense>

/// Internal imports
#include <gp_wind_regression/launch_parameters.h>
#include <gp_wind_regression/gp_model_base.h>

namespace wind_regression
{
class GPModelTypeConcatenated : public GPModelBase
{
private:
    size_t buffer_count_ = 0;
    libgp::GaussianProcess* gp_initial_;
    bool is_first_run_ = true;
    bool is_gp_initial_created_ = false;

public:
    /// Constructor
    /**
     * @brief
     */
    GPModelTypeConcatenated();

    GPModelTypeConcatenated(const RunParams& run_params, const GPParams& gp_params);

    /// Destructor
    /**
     * @brief
     */
    ~GPModelTypeConcatenated();

    /// Public Member Methods
    /**
     * @brief
     */
    void initialize(const FeatureDataStruct& feature_data, const double time_loop);

    /**
     * @brief
     */
    void core(const FeatureDataStruct& feature_data);

    /**
     * @brief
     */
    void createInitial(const FeatureDataStruct& feature_data, const double time_loop);

    /**
     * @brief
     */
    void loadInitial();
};

}  // end namespace wind_regression

#endif
