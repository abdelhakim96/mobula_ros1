/**
 * @file   data_struct.h
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief  Structs describing data for GP model.
 */

#ifndef _DATA_STRUCT_H_
#define _DATA_STRUCT_H_

/// External imports
#include <iostream>

/// Internal imports
#include <gp_wind_regression/utils.h>

namespace wind_regression
{
struct PoseStruct
{
    std::vector<double> position = std::vector<double>(3, 0.0);
    std::vector<double> attitude = std::vector<double>(3, 0.0);
};

struct TwistStruct
{
    std::vector<double> linear_velocity = std::vector<double>(3, 0.0);
    std::vector<double> angular_velocity = std::vector<double>(3, 0.0);
};

struct ImuStruct
{
    std::vector<double> linear_acceleration = std::vector<double>(3, 0.0);
    std::vector<double> orientation = std::vector<double>(4, 0.0);
    std::vector<double> angular_velocity = std::vector<double>(3, 0.0);
};

struct NmpcStruct
{
    std::vector<double> cmd_thrust = std::vector<double>(2, 0.0);  // Fz and Fz_scaled
    std::vector<double> cmd_attitude = std::vector<double>(3, 0.0);
};

struct AccelerationStruct
{
    std::vector<double> linear = std::vector<double>(3, 0.0);
    std::vector<std::vector<double>> linear_unfiltered;

    void filter(const std::vector<double>& acceleration)
    {
        for (int idx = 0; idx < linear.size(); idx++)
        {
            linear[idx] = utils::meanFilter(acceleration[idx], linear_unfiltered[idx]);
        }
    }
};

struct InputTargetDataStruct
{
    std::vector<double> input;
    double target;

    void setZeros(const uint& input_size)
    {
        input.resize(input_size, 0.0);
        target = 0.0;
    }

    void printInputTargetData(const std::string& name = "")
    {
        std::cout << "**********************************\n";
        utils::printStdVector(input, name + "input");
        utils::printStdVector({target}, name + "target");
        std::cout << "**********************************\n";
    }
};

struct FeatureDataStruct
{
    InputTargetDataStruct current_time;
    InputTargetDataStruct previous_time;

    void setZeros(const uint& input_size)
    {
        current_time.setZeros(input_size);
        previous_time.setZeros(input_size);
    }

    void printFeatureData()
    {
        current_time.printInputTargetData("current_time_");
        previous_time.printInputTargetData("previous_time_");
    }
};

struct MeasDataStruct
{
    PoseStruct pose_struct;
    TwistStruct twist_struct;
    ImuStruct imu_struct;
    NmpcStruct nmpc_struct;
    AccelerationStruct acceleration_struct;
    // Feature data could also be directly passed as measurements
    InputTargetDataStruct input_target_data_struct;

    void printPoseData()
    {
        utils::printStdVector(pose_struct.position, "position");
        utils::printStdVector(pose_struct.attitude, "attitude");
    }
    void printTwistData()
    {
        utils::printStdVector(twist_struct.linear_velocity, "linear_velocity");
        utils::printStdVector(twist_struct.angular_velocity, "angular_velocity");
    }
    void printImuData()
    {
        utils::printStdVector(imu_struct.linear_acceleration, "linear_acceleration");
        utils::printStdVector(imu_struct.orientation, "orientation");
        utils::printStdVector(imu_struct.angular_velocity, "angular_velocity");
    }
    void printNmpcData()
    {
        utils::printStdVector(nmpc_struct.cmd_thrust, "cmd_thrust");
        utils::printStdVector(nmpc_struct.cmd_attitude, "cmd_attitude");
    }
    void printAccelerationData()
    {
        utils::printStdVector(acceleration_struct.linear, "linear");
    }
    void printMeasData()
    {
        std::cout << "-----------------------------------\n";
        printPoseData();
        printTwistData();
        printImuData();
        printNmpcData();
        printAccelerationData();
        if (not input_target_data_struct.input.empty())
        {
            input_target_data_struct.printInputTargetData();
        }
        std::cout << "-----------------------------------\n";
    }
};

struct ModelDifferenceStruct
{
    double x, y, z;
    void printModelDifferenceValues()
    {
        utils::printStdVector(std::vector<double>{x, y, z}, "model_difference_values");
    }
};

struct RegressionDataStruct
{
    InputTargetDataStruct train;
    InputTargetDataStruct test;

    void popBack(const uint& num_elements)
    {
        uint i = 0;
        while (i++ < num_elements)
        {
            train.input.pop_back();
            test.input.pop_back();
        }
    }

    void pushFrontTrainInput(const std::vector<double> vector)
    {
        train.input.insert(train.input.begin(), vector.begin(), vector.end());
    }

    void pushFrontTestInput(const std::vector<double> vector)
    {
        test.input.insert(test.input.begin(), vector.begin(), vector.end());
    }

    void printRegressionData()
    {
        train.printInputTargetData("train_");
        test.printInputTargetData("test_");
    }
};

}  // namespace wind_regression

#endif
