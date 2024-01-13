/**
 * @file   regression_data_model.h
 * @author Mohit Mehndiratta
 * @date   Sep 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief
 */

#ifndef REGRESSION_DATA_MODEL_H
#define REGRESSION_DATA_MODEL_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports

/// Internal imports
#include <gp_wind_regression/launch_parameters.h>
#include <gp_wind_regression/data_struct.h>

namespace wind_regression
{
class RegressionDataModel
{
private:
    RegressionDataStruct regression_data_;
    RunParams run_params_;
    GPParams gp_params_;

    /**
     * @brief
     */
    void computeModelDifference(const MeasDataStruct& previous_data, const MeasDataStruct& current_data);

    /**
     * @brief
     */
    bool setFeatureDataForTypeModelDifference();

public:
    /// Constructor
    /**
     * @brief
     */
    RegressionDataModel();

    /**
     * @brief
     */
    RegressionDataModel(const RunParams& run_params, const GPParams& gp_params);

    /// Destructor
    /**
     * @brief
     */
    ~RegressionDataModel();

    /// Public Member Methods
    /**
     * @brief
     */
    bool prepareRegressionData(RegressionDataStruct& regression_data, const FeatureDataStruct& feature_data);

    /**
     * @brief
     */
    bool prepareRegressionData(const FeatureDataStruct& feature_data);

    /**
     * @brief
     */
    bool prepareFeatureDataForFuture(FeatureDataStruct& feature_data, const double& mean_value);

    /**
     * @brief
     */
    RegressionDataStruct getRegressionData();
};

}  // end namespace wind_regression

#endif
