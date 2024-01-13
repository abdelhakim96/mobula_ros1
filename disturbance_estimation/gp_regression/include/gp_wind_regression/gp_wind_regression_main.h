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

#ifndef GP_WIND_REGRESSION_MAIN_H
#define GP_WIND_REGRESSION_MAIN_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports
#include <Eigen/Dense>
/// Internal imports
#include <gp_wind_regression/gp_model.h>
#include <gp_wind_regression/feature_data_model.h>
#include <gp_wind_regression/data_struct.h>
#include <gp_wind_regression/utils.h>

namespace wind_regression
{
struct PrintFlagStuct
{
    int regression_switch_ = 0;
    int regression_start_ = 1;
};

struct StartFinishedFlagStruct
{
    bool started_ = false;
    bool finished_ = false;
};

class GPWindRegressionNode
{
private:
    /// Other variables
    bool is_initialization_failed_ = false;
    bool regression_stop_ = false;
    double time_last_, time_elapsed_;
    RunParams run_params_;
    GPParams gp_params_;
    MeasDataStruct previous_data_, current_data_;

    PrintFlagStuct print_flag_;
    StartFinishedFlagStruct started_finished_flag_;

    /// Instance of the logic class to be called from within the node
    GPModel* gp_model_;

    /// Instance of the feature data model class
    FeatureDataModel* feature_data_model_;

    /// Subscribers
    // Switch subscriber
    ros::Subscriber regression_on_sub_;
    // NMPC subscribers
    ros::Subscriber nmpc_cmd_attitude_sub_, nmpc_cmd_thrust_sub_;
    // UAV feedback subscribers
    ros::Subscriber vel_sub_, imu_sub_;
    // Feature data subscriber
    ros::Subscriber feature_data_sub_;

    /// Publishers
    // Switch publisher
    ros::Publisher prediction_init_pub_;
    // Estimation publishers
    ros::Publisher mean_pub_, variance_pub_, mean_p_2std_dev_pub_, mean_m_2std_dev_pub_;

    /// Topics
    // Subscriber topics
    std::string regression_on_sub_topic_, nmpc_cmd_attitude_sub_topic_, nmpc_cmd_thrust_sub_topic_, vel_sub_topic_,
        imu_sub_topic_, feature_data_sub_topic_;
    // Publisher topics
    std::string prediction_init_pub_topic_, mean_pub_topic_, variance_pub_topic_, mean_p_2std_dev_pub_topic_,
        mean_m_2std_dev_pub_topic_;

    /// Messages
    std_msgs::Bool regression_on_msg_, prediction_init_msg_;
    std_msgs::Float64MultiArray mean_msg_, variance_msg_, mean_p_2std_dev_msg_, mean_m_2std_dev_msg_;

    /// Initialize methods
    void initLaunchParameters(ros::NodeHandle& nh);
    void initSubscribers(ros::NodeHandle& nh);
    void initPublishers(ros::NodeHandle& nh);
    void initStdVectors();

    /// Callback methods
    void regressionOnCallback(const std_msgs::Bool::ConstPtr& msg);
    void nmpcCommandAttitudeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void nmpcCommandThrustCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void featureDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    /// Helper methods
    bool selectModelTypes();
    void initMultiArrayMessages();
    void publishOutputs();

public:
    /// Constructor
    GPWindRegressionNode(ros::NodeHandle& nh);

    /// Destructor
    ~GPWindRegressionNode();

    /// Helper Methods
    bool runNode();
};

}  // end namespace wind_regression

#endif
