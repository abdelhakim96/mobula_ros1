/**
 * @file   feature_data_model.h
 * @author Mohit Mehndiratta
 * @date   Sep 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief
 */

#ifndef FEATURE_DATA_MODEL_H
#define FEATURE_DATA_MODEL_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports

/// Internal imports
#include <gp_wind_regression/launch_parameters.h>
#include <gp_wind_regression/data_struct.h>

#define GRAVITATIONAL_CONST 9.81
#define NUM_MODEL_DIFFERENCE_FEATURES 1
#define NUM_STATES_FEATURES 1
#define NUM_CONTROLS_FEATURES 1

namespace wind_regression
{
class FeatureDataModel
{
private:
    FeatureDataStruct feature_data_;
    RunParams run_params_;
    GPParams gp_params_;

    ModelDifferenceStruct model_difference_values;

    /**
     * @brief
     */
    void computeModelDifference(const MeasDataStruct& previous_data, const MeasDataStruct& current_data);

    /**
     * @brief
     */
    bool setFeatureData(const InputTargetDataStruct& input_target_data);

    /**
     * @brief
     */
    bool setFeatureData(const MeasDataStruct& measurement_data);

    /**
     * @brief
     */
    bool updateFeatureDataForInputDataType1();

public:
    /// Constructor
    /**
     * @brief
     */
    FeatureDataModel();

    /**
     * @brief
     */
    FeatureDataModel(const RunParams& run_params, const GPParams& gp_params);

    /// Destructor
    /**
     * @brief
     */
    ~FeatureDataModel();

    /// Public Member Methods
    /**
     * @brief
     */
    FeatureDataStruct computeAndGetFeatureData(const MeasDataStruct& previous_data, const MeasDataStruct& current_data);

    /**
     * @brief
     */
    FeatureDataStruct getFeatureData();
};

}  // end namespace wind_regression

#endif
