/**
 * @file   regression_data_model.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/regression_data_model.h>

namespace wind_regression
{
RegressionDataModel::RegressionDataModel(const RunParams& run_params, const GPParams& gp_params)
    : run_params_(run_params)
    , gp_params_(gp_params)
{
    // Initialize regression_data_;
    regression_data_.train.input.resize(gp_params_.input_data_params_.num_inputs_, 0.0);
    regression_data_.test.input.resize(gp_params_.input_data_params_.num_inputs_, 0.0);

    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "RegressionDataModelLOG: Constructor is called!\n";
    }
}

RegressionDataModel::~RegressionDataModel()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "RegressionDataModelLOG: Destructor is called!\n";
    }
}

bool RegressionDataModel::prepareRegressionData(RegressionDataStruct& regression_data,
                                                const FeatureDataStruct& feature_data)
{
    regression_data.popBack(gp_params_.input_data_params_.num_features_);
    regression_data.pushFrontTrainInput(feature_data.previous_time.input);
    regression_data.pushFrontTestInput(feature_data.current_time.input);
    regression_data.train.target = feature_data.current_time.target;
    if (regression_data.train.input.size() != regression_data.test.input.size())
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**RegressionDataModelERROR: Dimension of training inputs and test inputs should be same!\n";
        }
        return 0;
    }

    return 1;
}

bool RegressionDataModel::prepareRegressionData(const FeatureDataStruct& feature_data)
{
    return prepareRegressionData(regression_data_, feature_data);
}

// TODO: think about moving this to FeatureDataModel
bool RegressionDataModel::prepareFeatureDataForFuture(FeatureDataStruct& feature_data, const double& mean_value)
{
    feature_data.previous_time = feature_data.current_time;
    feature_data.current_time.input.clear();
    switch (gp_params_.input_data_params_.gp_input_data_type_)
    {
        case GPInputDataType::TYPE0:
        {
            feature_data.current_time.input.push_back(mean_value);
        }
        break;
        case GPInputDataType::TYPE1:
        {
            feature_data.current_time.input.push_back(mean_value);
            feature_data.current_time.input.push_back(mean_value - feature_data.previous_time.input[0]);
        }
        break;
        default:
        {
            if (run_params_.debug_flag_ >= 0)
            {
                std::cout << "**GPModelBaseERROR: Enter a valid GPInputDataType!\n";
            }
            return 0;
        }
        break;
    }
    // For this feature data type, target is the same as input
    feature_data.current_time.target = feature_data.current_time.input[0];
}

RegressionDataStruct RegressionDataModel::getRegressionData()
{
    return regression_data_;
}

}  // namespace wind_regression
