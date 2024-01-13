/**
 * @file   feature_data_model.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/feature_data_model.h>

namespace wind_regression
{
FeatureDataModel::FeatureDataModel(const RunParams& run_params, const GPParams& gp_params)
    : run_params_(run_params)
    , gp_params_(gp_params)
{
    // Initialize feature_data_
    feature_data_.previous_time.input.resize(gp_params_.input_data_params_.num_features_, 0.0);
    feature_data_.current_time = feature_data_.previous_time;

    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "FeatureDataModelLOG: Constructor is called!\n";
    }
}

FeatureDataModel::~FeatureDataModel()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "FeatureDataModelLOG: Destructor is called!\n";
    }
}

void FeatureDataModel::computeModelDifference(const MeasDataStruct& previous_data, const MeasDataStruct& current_data)
{
    model_difference_values.x =
        current_data.acceleration_struct.linear[0] -
        previous_data.twist_struct.angular_velocity[2] * previous_data.twist_struct.linear_velocity[1] +
        previous_data.twist_struct.angular_velocity[1] * previous_data.twist_struct.linear_velocity[2] -
        GRAVITATIONAL_CONST * sin(previous_data.nmpc_struct.cmd_attitude[1]);

    model_difference_values.y =
        current_data.acceleration_struct.linear[1] -
        previous_data.twist_struct.angular_velocity[0] * previous_data.twist_struct.linear_velocity[2] +
        previous_data.twist_struct.angular_velocity[2] * previous_data.twist_struct.linear_velocity[0] +
        GRAVITATIONAL_CONST * sin(previous_data.nmpc_struct.cmd_attitude[0]) *
            cos(previous_data.nmpc_struct.cmd_attitude[1]);

    model_difference_values.z =
        current_data.acceleration_struct.linear[2] -
        previous_data.twist_struct.angular_velocity[1] * previous_data.twist_struct.linear_velocity[0] +
        previous_data.twist_struct.angular_velocity[0] * previous_data.twist_struct.linear_velocity[1] +
        GRAVITATIONAL_CONST * cos(previous_data.nmpc_struct.cmd_attitude[0]) *
            cos(previous_data.nmpc_struct.cmd_attitude[1]) -
        (1 / run_params_.mass_) * previous_data.nmpc_struct.cmd_thrust[0];
}

bool FeatureDataModel::updateFeatureDataForInputDataType1()
{
    uint idx = 0;
    while (idx < (uint)gp_params_.input_data_params_.num_features_ / 2)
    {
        feature_data_.current_time.input.push_back(feature_data_.current_time.input[idx] -
                                                   feature_data_.previous_time.input[idx]);
        idx++;
    }
    if (feature_data_.current_time.input.size() != gp_params_.input_data_params_.num_features_)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout
                << "**FeatureDataModelERROR: Dimension inconsistency in updating feature data for input data type1!\n";
        }
        return 0;
    }
    return 1;
}

FeatureDataStruct FeatureDataModel::computeAndGetFeatureData(const MeasDataStruct& previous_data,
                                                             const MeasDataStruct& current_data)
{
    feature_data_.previous_time = feature_data_.current_time;
    computeModelDifference(previous_data, current_data);

    switch (gp_params_.input_data_params_.feature_data_model_type_)
    {
        case FeatureDataModelType::SUBSCRIBE:
        {
            if (not setFeatureData(current_data.input_target_data_struct))
            {
                return FeatureDataStruct();
            }
        }
        break;
        case FeatureDataModelType::COMPUTE:
        {
            if (not setFeatureData(current_data))
            {
                return FeatureDataStruct();
            }
        }
        break;
    }

    if (gp_params_.input_data_params_.gp_input_data_type_ == GPInputDataType::TYPE1)
    {
        if (not updateFeatureDataForInputDataType1())
        {
            return FeatureDataStruct();
        }
    }

    if (run_params_.debug_flag_ > 2)
    {
        feature_data_.printFeatureData();
    }

    return feature_data_;
}

bool FeatureDataModel::setFeatureData(const InputTargetDataStruct& input_target_data)
{
    if (input_target_data.input.size() != gp_params_.input_data_params_.num_features_)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout
                << "**FeatureDataModelERROR: Subscribed input_target_data is of incorrect size for feature_data !\n";
        }
        return 0;
    }
    feature_data_.current_time = input_target_data;
    return 1;
}

bool FeatureDataModel::setFeatureData(const MeasDataStruct& measurement_data)
{
    auto is_feature_data_type_present = [&](const FeatureDataType& feature_data_type) {
        auto result = std::find(gp_params_.input_data_params_.feature_data_type_.begin(),
                                gp_params_.input_data_params_.feature_data_type_.end(),
                                feature_data_type);
        return result != gp_params_.input_data_params_.feature_data_type_.end();
    };
    auto input_data = [&](const double& model_diff_value, const uint& state_idx, const uint& control_idx) {
        std::vector<double> data;
        if (is_feature_data_type_present(FeatureDataType::MODELDIFFERENCE))
        {
            data.push_back(model_diff_value);
        }
        if (is_feature_data_type_present(FeatureDataType::STATES))
        {
            //data.push_back(measurement_data.pose_struct.position[state_idx]);
            data.push_back(measurement_data.twist_struct.linear_velocity[state_idx]);
        }
        if (is_feature_data_type_present(FeatureDataType::CONTROLS))
        {
            if (run_params_.gp_direction_type_ == GPDirectionType::Z)
            {
                data.push_back(measurement_data.nmpc_struct.cmd_thrust[control_idx]);
            }
            else
            {
                data.push_back(measurement_data.nmpc_struct.cmd_attitude[control_idx]);
            }
        }
        return data;
    };
    feature_data_.current_time.input.clear();
    switch (run_params_.gp_direction_type_)
    {
        case GPDirectionType::X:
        {
            feature_data_.current_time.input = input_data(model_difference_values.x, 0, 1);
        }
        break;
        case GPDirectionType::Y:
        {
            feature_data_.current_time.input = input_data(model_difference_values.y, 1, 0);
        }
        break;
        case GPDirectionType::Z:
        {
            feature_data_.current_time.input = input_data(model_difference_values.z, 2, 0);
        }
        break;
        default:
        {
            if (run_params_.debug_flag_ >= 0)
            {
                std::cout << "**FeatureDataModelERROR: Enter a valid switch_xyz number (x:0, y:1, z:2)!\n";
            }
            return 0;
        }
        break;
    }

    const bool is_input_of_correct_size =
        feature_data_.current_time.input.size() ==
                (gp_params_.input_data_params_.gp_input_data_type_ == GPInputDataType::TYPE1)
            ? (uint)gp_params_.input_data_params_.num_features_ / 2
            : gp_params_.input_data_params_.num_features_;

    if (not is_input_of_correct_size)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**FeatureDataModelERROR: Generated feature_data_input is of incorrect size!\n";
        }
        return 0;
    }

    // For this feature data type, target is the same as input
    feature_data_.current_time.target = feature_data_.current_time.input[0];

    return 1;
}

FeatureDataStruct FeatureDataModel::getFeatureData()
{
    return feature_data_;
}

}  // namespace wind_regression
