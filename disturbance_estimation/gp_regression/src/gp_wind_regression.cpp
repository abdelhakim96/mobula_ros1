/**
 * @file   gp_wind_regression_main.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/gp_wind_regression_main.h>

namespace wind_regression
{
/// Constructor
GPWindRegressionNode::GPWindRegressionNode(ros::NodeHandle& nh)
{
    run_params_.ros_namespace_ = ros::this_node::getNamespace();
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: ROS namespace = " << run_params_.ros_namespace_.c_str() << "\n";
    }

    initLaunchParameters(nh);
    initSubscribers(nh);
    initPublishers(nh);
    initMultiArrayMessages();
    initStdVectors();

    if (!selectModelTypes())
    {
        is_initialization_failed_ = true;
    }

    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Constructor is called!\n";
    }
}

/// Destructor
GPWindRegressionNode::~GPWindRegressionNode()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Destructor is called!\n";
    }
}

void GPWindRegressionNode::initLaunchParameters(ros::NodeHandle& nh)
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Initializing launch paramters!\n";
    }

    // Run parameters
    nh.param<int>(run_params_.ros_namespace_ + "/debug_flag", run_params_.debug_flag_, 1);
    nh.param<int>(run_params_.ros_namespace_ + "/ros_loop_rate", run_params_.ros_loop_rate_, 20);
    int direction_type;
    nh.param<int>(run_params_.ros_namespace_ + "/switch_xyz", direction_type, 0);
    run_params_.gp_direction_type_ = (GPDirectionType)direction_type;
    nh.param<int>(run_params_.ros_namespace_ + "/filter_window", run_params_.filter_window_, 1);
    nh.param<double>(run_params_.ros_namespace_ + "/mass", run_params_.mass_, 2.0);

    // Subscribed topic names
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/regression_on_sub_topic", regression_on_sub_topic_, "/regression_on");
    nh.param<std::string>(run_params_.ros_namespace_ + "/nmpc_cmd_attitude_sub_topic",
                          nmpc_cmd_attitude_sub_topic_,
                          "/outer_nmpc_cmd/rpy");
    nh.param<std::string>(run_params_.ros_namespace_ + "/nmpc_cmd_thrust_sub_topic",
                          nmpc_cmd_thrust_sub_topic_,
                          "/outer_nmpc_cmd/Fz_FzScaled");
    nh.param<std::string>(run_params_.ros_namespace_ + "/vel_sub_topic", vel_sub_topic_, "/mavros/mocap/velocity_body");
    nh.param<std::string>(run_params_.ros_namespace_ + "/imu_sub_topic", imu_sub_topic_, "/mavros/imu/data");
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/feature_data_sub_topic", feature_data_sub_topic_, "/gp_feature_data");

    // Published topic names
    nh.param<std::string>(run_params_.ros_namespace_ + "/prediction_init_pub_topic",
                          prediction_init_pub_topic_,
                          "/gp_disturb_reg/predInit/x");
    nh.param<std::string>(run_params_.ros_namespace_ + "/mean_pub_topic", mean_pub_topic_, "/gp_disturb_reg/mu/x");
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/variance_pub_topic", variance_pub_topic_, "/gp_disturb_reg/var/x");
    nh.param<std::string>(run_params_.ros_namespace_ + "/mean_p_2std_dev_pub_topic",
                          mean_p_2std_dev_pub_topic_,
                          "/gp_disturb_reg/mu_p_2std_dev/x");
    nh.param<std::string>(run_params_.ros_namespace_ + "/mean_m_2std_dev_pub_topic",
                          mean_m_2std_dev_pub_topic_,
                          "/gp_disturb_reg/mu_m_2std_dev/x");

    // GP parameters
    gp_params_.filepath_gp_initial_ = ros::package::getPath("gp_wind_regression") + "/data/gp/";
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/file_gp_initial", gp_params_.filename_gp_initial_, "gp_initial_x.dat");
    gp_params_.file_gp_initial_ = gp_params_.filepath_gp_initial_ + gp_params_.filename_gp_initial_;
    nh.param<bool>(run_params_.ros_namespace_ + "/create_new_gp_initial", gp_params_.create_new_gp_initial_, true);
    nh.param<bool>(run_params_.ros_namespace_ + "/use_gp_initial", gp_params_.use_gp_initial_, true);
    nh.param<bool>(run_params_.ros_namespace_ + "/use_sparse_gp_initial", gp_params_.use_sparse_gp_initial_, false);
    int gp_model_type, gp_input_data_type, feature_data_model_type;
    nh.param<int>(run_params_.ros_namespace_ + "/gp_model_type", gp_model_type, 0);
    nh.param<int>(run_params_.ros_namespace_ + "/gp_input_data_type", gp_input_data_type, 0);
    nh.param<int>(run_params_.ros_namespace_ + "/feature_data_model_type", feature_data_model_type, 1);
    gp_params_.gp_model_type_ = (GPModelType)gp_model_type;
    int max_initial_rec_time_sec, num_window_points_past, num_predict_points_future;
    nh.param<int>(run_params_.ros_namespace_ + "/max_initial_rec_time_sec", max_initial_rec_time_sec, 15);
    nh.param<int>(run_params_.ros_namespace_ + "/num_window_points_past", num_window_points_past, 200);
    nh.param<int>(run_params_.ros_namespace_ + "/num_predict_points_future", num_predict_points_future, 30);
    gp_params_.max_initial_rec_time_sec_ = max_initial_rec_time_sec;
    gp_params_.num_window_points_past_ = num_window_points_past;
    gp_params_.num_predict_points_future_ = num_predict_points_future;
    nh.param<double>(run_params_.ros_namespace_ + "/rec_rate_factor", gp_params_.rec_rate_factor_, 0.7);
    nh.param<double>(run_params_.ros_namespace_ + "/sizefactor_sparse_gp_initial", gp_params_.sizefactor_sparse_gp_initial_, 1.0);

    
    nh.param<double>(run_params_.ros_namespace_ + "/lambda", gp_params_.lambda_, 1.0);
    //gp_params_.lambda_ = lambda;

    if (gp_params_.sizefactor_sparse_gp_initial_ > 1)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**NodeWARN: Value greater than 1 is input for "
                         "sizefactor_sparse_gp_initial.\n\t "
                         "sizefactor_sparse_gp_initial = 1 is enforced.\n";
        }
        gp_params_.sizefactor_sparse_gp_initial_ = 1;
    }

    gp_params_.input_data_params_.gp_input_data_type_ = (GPInputDataType)gp_input_data_type;
    gp_params_.input_data_params_.feature_data_model_type_ = (FeatureDataModelType)feature_data_model_type;
    if (gp_params_.input_data_params_.feature_data_model_type_ == FeatureDataModelType::SUBSCRIBE)
    {
        int num_features;
        nh.param<int>(run_params_.ros_namespace_ + "/num_features", num_features, 1);
        gp_params_.input_data_params_.num_features_ = num_features;

        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**NodeWARN: Future prediction for SUBSCRIBE feature data model type is not supported.\n"
                         "num_predict_points_future_ = 1 is enforced.\n";
        }
        gp_params_.num_predict_points_future_ = 1;
    }
    else
    {
        bool use_model_difference, use_states, use_controls;
        nh.param<bool>(run_params_.ros_namespace_ + "/use_model_difference", use_model_difference, true);
        nh.param<bool>(run_params_.ros_namespace_ + "/use_states", use_states, false);
        nh.param<bool>(run_params_.ros_namespace_ + "/use_controls", use_controls, false);
        if (use_model_difference)
        {
            gp_params_.input_data_params_.feature_data_type_.push_back(FeatureDataType::MODELDIFFERENCE);
        }
        if (use_states)
        {
            gp_params_.input_data_params_.feature_data_type_.push_back(FeatureDataType::STATES);
        }
        if (use_controls)
        {
            gp_params_.input_data_params_.feature_data_type_.push_back(FeatureDataType::CONTROLS);
        }

        gp_params_.input_data_params_.num_features_ = use_model_difference * NUM_MODEL_DIFFERENCE_FEATURES +
                                                      use_states * NUM_STATES_FEATURES +
                                                      use_controls * NUM_CONTROLS_FEATURES;

        if (use_states or use_controls)
        {
            if (run_params_.debug_flag_ >= 0)
            {
                std::cout << "**NodeWARN: Future prediction in only supported for feature data type MODELDIFFERENCE.\n"
                             "num_predict_points_future_ = 1 is enforced.\n";
            }
            gp_params_.num_predict_points_future_ = 1;
        }
    }
    switch (gp_params_.input_data_params_.gp_input_data_type_)
    {
        case GPInputDataType::TYPE0:
        {
            gp_params_.input_data_params_.num_features_ *= 1;
        }
        break;

        case GPInputDataType::TYPE1:
        {
            gp_params_.input_data_params_.num_features_ *= 2;
        }
        break;
    }
    int data_history;
    nh.param<int>(run_params_.ros_namespace_ + "/data_history", data_history, 5);
    gp_params_.input_data_params_.data_history_ = data_history;
    gp_params_.input_data_params_.num_inputs_ =
        gp_params_.input_data_params_.num_features_ * gp_params_.input_data_params_.data_history_;

    gp_params_.covariance_function_params_.ell_t0_.setConstant(gp_params_.input_data_params_.num_inputs_, 1.0);
    gp_params_.covariance_function_params_.sx_t0_.setConstant(gp_params_.input_data_params_.num_inputs_, 0.0 * 0.0);

    nh.param<double>(run_params_.ros_namespace_ + "/sf_t0", gp_params_.covariance_function_params_.sf_t0_, 1.0);
    nh.param<double>(run_params_.ros_namespace_ + "/sn_t0", gp_params_.covariance_function_params_.sn_t0_, 1.0);
    nh.param<int>(run_params_.ros_namespace_ + "/max_gradient_updates", gp_params_.max_gradient_updates_, 100);
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/covfun1", gp_params_.covariance_function_params_.covfun1_, "CovSEard");
    nh.param<std::string>(
        run_params_.ros_namespace_ + "/covfun2", gp_params_.covariance_function_params_.covfun2_, "CovNoise");
    gp_params_.covariance_function_params_.final_covfun_ = "CovSum ( " +
                                                           gp_params_.covariance_function_params_.covfun1_ + ", " +
                                                           gp_params_.covariance_function_params_.covfun2_ + ")";
    int hyper_optimize_method;
    nh.param<int>(run_params_.ros_namespace_ + "/hyper_optimize_method", hyper_optimize_method, 0);
    gp_params_.hyper_optimize_method_ = (HyperOptimizeMethod)hyper_optimize_method;

    nh.param<double>(run_params_.ros_namespace_ + "/mean_t0", gp_params_.mean_t0_, 0.0);
    nh.param<double>(run_params_.ros_namespace_ + "/variance_t0", gp_params_.variance_t0_, 100.0);
}

void GPWindRegressionNode::initSubscribers(ros::NodeHandle& nh)
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Initializing subscribers!\n";
    }
    /// Subscribers
    // Switch subscriber
    regression_on_sub_ = nh.subscribe<std_msgs::Bool>(
        regression_on_sub_topic_, 1, &wind_regression::GPWindRegressionNode::regressionOnCallback, this);
    // NMPC subscribers
    nmpc_cmd_attitude_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
        nmpc_cmd_attitude_sub_topic_, 1, &wind_regression::GPWindRegressionNode::nmpcCommandAttitudeCallback, this);
    nmpc_cmd_thrust_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
        nmpc_cmd_thrust_sub_topic_, 1, &wind_regression::GPWindRegressionNode::nmpcCommandThrustCallback, this);
    // UAV feedback subscribers
    vel_sub_ = nh.subscribe<geometry_msgs::TwistStamped>(
        vel_sub_topic_, 1, &wind_regression::GPWindRegressionNode::velocityCallback, this);
    imu_sub_ =
        nh.subscribe<sensor_msgs::Imu>(imu_sub_topic_, 1, &wind_regression::GPWindRegressionNode::imuCallback, this);
    if (gp_params_.input_data_params_.feature_data_model_type_ == FeatureDataModelType::SUBSCRIBE)
    {
        feature_data_sub_ = nh.subscribe<std_msgs::Float64MultiArray>(
            feature_data_sub_topic_, 1, &wind_regression::GPWindRegressionNode::featureDataCallback, this);
    }
}

void GPWindRegressionNode::initPublishers(ros::NodeHandle& nh)
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Initializing publishers!\n";
    }
    /// Publishers
    // Switch publisher
    prediction_init_pub_ = nh.advertise<std_msgs::Bool>(prediction_init_pub_topic_, 1, true);
    // Estimation publishers
    mean_pub_ = nh.advertise<std_msgs::Float64MultiArray>(mean_pub_topic_, 1, true);
    variance_pub_ = nh.advertise<std_msgs::Float64MultiArray>(variance_pub_topic_, 1, true);
    mean_p_2std_dev_pub_ = nh.advertise<std_msgs::Float64MultiArray>(mean_p_2std_dev_pub_topic_, 1, true);
    mean_m_2std_dev_pub_ = nh.advertise<std_msgs::Float64MultiArray>(mean_m_2std_dev_pub_topic_, 1, true);
}

void GPWindRegressionNode::initMultiArrayMessages()
{
    utils::initMultiArrayMessage(mean_msg_, gp_params_.num_predict_points_future_, "mean");
    utils::initMultiArrayMessage(variance_msg_, gp_params_.num_predict_points_future_, "variance");
    utils::initMultiArrayMessage(mean_p_2std_dev_msg_, gp_params_.num_predict_points_future_, "mean_p_2std_dev");
    utils::initMultiArrayMessage(mean_m_2std_dev_msg_, gp_params_.num_predict_points_future_, "mean_m_2std_dev");
}

void GPWindRegressionNode::initStdVectors()
{
    current_data_.acceleration_struct.linear_unfiltered.resize(current_data_.acceleration_struct.linear.size(),
                                                               std::vector<double>(run_params_.filter_window_, 0.0));
}

/// Update the callbacks
void GPWindRegressionNode::regressionOnCallback(const std_msgs::Bool::ConstPtr& msg)
{
    regression_on_msg_ = *msg;
}
void GPWindRegressionNode::nmpcCommandAttitudeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    utils::extractMultiArrayMsg(current_data_.nmpc_struct.cmd_attitude, msg->data.begin(), msg->data.end());
}
void GPWindRegressionNode::nmpcCommandThrustCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    utils::extractMultiArrayMsg(current_data_.nmpc_struct.cmd_thrust, msg->data.begin(), msg->data.end());
}
void GPWindRegressionNode::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_data_.twist_struct.linear_velocity = {msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z};
    current_data_.twist_struct.angular_velocity = {msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
}

void GPWindRegressionNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    current_data_.imu_struct.linear_acceleration = {
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    //    // Compensation for gravity
    current_data_.imu_struct.linear_acceleration.back() -= GRAVITATIONAL_CONST;
    current_data_.imu_struct.orientation = {
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w};
    current_data_.imu_struct.angular_velocity = {
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};

    // filter acceleration data
    current_data_.acceleration_struct.filter(current_data_.imu_struct.linear_acceleration);
}

void GPWindRegressionNode::featureDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // Msg convention: first element is target value, followed by inputs
    current_data_.input_target_data_struct.target = msg->data.front();
    utils::extractMultiArrayMsg(current_data_.input_target_data_struct.input, msg->data.begin() + 1, msg->data.end());
}

bool GPWindRegressionNode::selectModelTypes()
{
    // select the corresponding GP Model
    switch (gp_params_.gp_model_type_)
    {
        case GPModelType::CONCATENATED:
        {
            gp_model_ = new GPModel(run_params_, gp_params_);
        }
        break;
        case GPModelType::OTHER:
        {
            if (run_params_.debug_flag_ >= 0)
            {
                std::cout << "**NodeERROR: Implement the method first!\nexiting...\n";
            }
            return 0;
        }
        break;
    }
    if (gp_model_ == nullptr)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**NodeERROR: Created gp_model_ is a null pointer.\n";
        }
        return 0;
    }

    // Set the feature data model
    feature_data_model_ = new FeatureDataModel(run_params_, gp_params_);
    if (feature_data_model_ == nullptr)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**NodeERROR: Created feature_data_model_ is a null pointer.\n";
        }
        return 0;
    }

    return 1;
}

void GPWindRegressionNode::publishOutputs()
{
    prediction_init_msg_.data = gp_model_->getGPModelBase()->isRegressionInitialized();
    prediction_init_pub_.publish(prediction_init_msg_);

    mean_msg_.data.clear();
    std::vector<double> mean_vec = gp_model_->getGPModelBase()->getPredictionData().mean_unstandardized_;
    mean_msg_.data.insert(mean_msg_.data.end(), mean_vec.begin(), mean_vec.end());
    mean_pub_.publish(mean_msg_);

    variance_msg_.data.clear();
    std::vector<double> variance_vec = gp_model_->getGPModelBase()->getPredictionData().variance_unstandardized_;
    variance_msg_.data.insert(variance_msg_.data.end(), variance_vec.begin(), variance_vec.end());
    variance_pub_.publish(variance_msg_);

    mean_p_2std_dev_msg_.data.clear();
    std::vector<double> mean_p_2std_dev_vec = gp_model_->getGPModelBase()->getPredictionData().mean_p_2std_dev_;
    mean_p_2std_dev_msg_.data.insert(
        mean_p_2std_dev_msg_.data.end(), mean_p_2std_dev_vec.begin(), mean_p_2std_dev_vec.end());
    mean_p_2std_dev_pub_.publish(mean_p_2std_dev_msg_);

    mean_m_2std_dev_msg_.data.clear();
    std::vector<double> mean_m_2std_dev_vec = gp_model_->getGPModelBase()->getPredictionData().mean_m_2std_dev_;
    mean_m_2std_dev_msg_.data.insert(
        mean_m_2std_dev_msg_.data.end(), mean_m_2std_dev_vec.begin(), mean_m_2std_dev_vec.end());
    mean_m_2std_dev_pub_.publish(mean_m_2std_dev_msg_);
}

bool GPWindRegressionNode::runNode()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "NodeLOG: Running Node!\n";
    }
    /// Set the loop control rate
    ros::Rate loop_rate(run_params_.ros_loop_rate_);
    ros::Rate initialization_loop_rate(gp_params_.rec_rate_factor_ * run_params_.ros_loop_rate_);

    // Initial wait for 1sec
    for (int i = 0; i < (int)1 * run_params_.ros_loop_rate_; ++i)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok() && !regression_stop_)
    {
        time_last_ = ros::Time::now().toSec();

        if (!regression_on_msg_.data && print_flag_.regression_switch_ == 0)
        {
            if (run_params_.debug_flag_ > 1)
            {
                std::cout << "NodeLOG: Waiting for regression switch to begin!\n";
            }
            print_flag_.regression_switch_ = 1;
        }
        //        else if (regression_on_msg_.data && print_flag_.regression_switch_
        //        == 1)
        //        {
        //            if (run_params_.debug_flag_ > 1)
        //            {
        //                std::cout << "NodeLOG: Regression switch turned on!\n";
        //            }
        //            print_flag_.regression_switch_ = 0;
        //        }

        while (ros::ok() && regression_on_msg_.data && !regression_stop_)
        {
            time_elapsed_ = ros::Time::now().toSec() - time_last_;

            if (print_flag_.regression_switch_ == 1)
            {
                if (run_params_.debug_flag_ > 1)
                {
                    std::cout << "NodeLOG: Regression switch turned on!\n";
                }
                print_flag_.regression_switch_ = 0;
            }
            feature_data_model_->computeAndGetFeatureData(previous_data_, current_data_);
            if (!gp_model_->getGPModelBase()->isRegressionInitialized())
            {
                gp_model_->getGPModelBase()->initialize(feature_data_model_->getFeatureData(), time_elapsed_);
            }
            else
            {
                gp_model_->getGPModelBase()->core(feature_data_model_->getFeatureData());
            }

            if (std::isnan(gp_model_->getGPModelBase()->getPredictionData().mean_.front()) ||
                std::isnan(gp_model_->getGPModelBase()->getPredictionData().mean_.back()))
            {
                if (run_params_.debug_flag_ >= 0)
                {
                    std::cout << "**NodeERROR: GP Regression ERROR at time = " << time_elapsed_ << " (sec) \n";
                }
                return 0;
            }

            if (run_params_.debug_flag_ > 1 && gp_model_->getGPModelBase()->isRegressionInitialized() &&
                std::fmod(time_elapsed_ - std::floor(time_elapsed_), 1.0) < 1.0 / run_params_.ros_loop_rate_)
            {
                std::cout << "NodeLOG: Elapsed time: " << (int)time_elapsed_ << " (sec)!\n";
            }

            previous_data_ = current_data_;

            publishOutputs();
            ros::spinOnce();
            // TODO: find a better solution for different record rate during initialization
            if (!gp_model_->getGPModelBase()->isRegressionInitialized())
            {
                initialization_loop_rate.sleep();
            }
            else
            {
                loop_rate.sleep();
            }
        }

        previous_data_ = current_data_;

        publishOutputs();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}

}  // namespace wind_regression
