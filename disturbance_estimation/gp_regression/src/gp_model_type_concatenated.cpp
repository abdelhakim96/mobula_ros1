/**
 * @file   gp_model_type_concatenated.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/gp_model_type_concatenated.h>

namespace wind_regression
{
GPModelTypeConcatenated::GPModelTypeConcatenated(const RunParams& run_params, const GPParams& gp_params)
    : GPModelBase(run_params, gp_params)
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelTypeConcatenatedLOG: Constructor is called!\n";
    }
}

GPModelTypeConcatenated::GPModelTypeConcatenated()
{
    fprintf(stderr, "This (GPModelTypeConcatenated::GPModelTypeConcatenated()) should never be called");
}

GPModelTypeConcatenated::~GPModelTypeConcatenated()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelTypeConcatenatedLOG: Destructor is called!\n";
    }
}

void GPModelTypeConcatenated::initialize(const FeatureDataStruct& feature_data, const double time_loop)
{
    if (gp_params_.use_gp_initial_ && !is_gp_initial_created_)
    {
        if (gp_params_.create_new_gp_initial_)
        {
            createInitial(feature_data, time_loop);
        }
        else
        {
            loadInitial();
        }

        if (is_gp_initial_created_)
        {
            gp_ = new libgp::GaussianProcess(*gp_initial_);
            if (run_params_.debug_flag_ > 1)
            {
                std::cout << "GPModelTypeConcatenatedLOG: gp_initial created!\n";
            }
        }
    }
    else if (!gp_params_.use_gp_initial_ && !is_gp_initial_created_)
    {
        if (run_params_.debug_flag_ >= 1)
        {
            std::cout << "**GPModelTypeConcatenatedWARN: gp_initial (LSTM) is not utilized for regression!\n";
        }
        is_gp_initial_created_ = true;
    }
    else  // Regression initialization
    {
        // TODO: check how gp_ is initialized when use_gp_initial_ is false?
        if (buffer_count_++ < gp_params_.num_window_points_past_)
        {
            if (gp_params_.use_gp_initial_)
            {
                regression_data_model_->prepareRegressionData(getStandardizedFeatureData(feature_data));
                addPattern(gp_);
            }
            else
            {
                regression_data_model_->prepareRegressionData(feature_data);
                if (buffer_count_ > gp_params_.input_data_params_.data_history_)
                {
                    addPattern(gp_);
                }
            }
        }
        else
        {
            if (!gp_params_.use_gp_initial_)
            {
                standardizeAndOptimize(gp_);
            }
            gp_->update_params_noisy_inp(gp_params_.covariance_function_params_.sx_t0_);
            is_regression_initialized_ = true;
            if (run_params_.debug_flag_ > 1)
            {
                std::cout << "GPModelTypeConcatenatedLOG: Regression is initialized!\n";
            }
        }
    }
}
//implement forgetting here!!
void GPModelTypeConcatenated::core(const FeatureDataStruct& feature_data)
{
    regression_data_model_->prepareRegressionData(getStandardizedFeatureData(feature_data));
    
    // add training patterns with removal from the front of the moving window
    if (gp_params_.use_gp_initial_)
    {
        // With gp_initial_ in use, the front of the moving window is after gp_initial_.
        addPatternwithRemoval(gp_, gp_initial_->get_sampleset_size());
    }
    else
    {
        addPatternwithRemoval(gp_, 0);
    }

    uint prediction_idx = 0;
     
    //Calculate error in prediction for forgetting factor

    double pred_error = abs(feature_data.current_time.target- prediction_data_.mean_[prediction_idx]);
    
    //double lamb = 0.90 + 0.1 / (1 + 0.01*std::exp(pred_error));
    
    double lamb = 1.0; 
    lamb =gp_params_.lambda_;

    //lamb =0.97;

    std::cout<<"lambda: " <<lamb<<std::endl;
    //lamb=1.0;
    // Computes mean and variance for current time
    computeMeanVariance(gp_,
                        0.0,
                        prediction_idx++,
                        utils::getarrayFromStdVector(regression_data_model_->getRegressionData().test.input),
                        lamb
                        );

    // Gets into future prediction only if the window size > 1
    if (gp_params_.num_predict_points_future_ > 1)
    {
        // Creates new GP for future window
        libgp::GaussianProcess* gp_future_window = new libgp::GaussianProcess(*gp_);
        RegressionDataStruct regression_data_future_window = regression_data_model_->getRegressionData();
        FeatureDataStruct feature_data_future_window = feature_data;

        // Adding recursive data to the rest of gp_future_window
        while (prediction_idx++ < gp_params_.num_predict_points_future_)
        {
            computeMeanVarianceNoisyInputs(gp_future_window,
                                           0.0,
                                           prediction_idx,
                                           utils::getarrayFromStdVector(regression_data_future_window.test.input));

            regression_data_model_->prepareFeatureDataForFuture(feature_data_future_window,
                                                                prediction_data_.mean_[prediction_idx]);
            regression_data_model_->prepareRegressionData(regression_data_future_window, feature_data_future_window);
            addPattern(gp_future_window, regression_data_future_window);
        }
        // TODO: check if needed?
        delete gp_future_window;
    }
}

void GPModelTypeConcatenated::createInitial(const FeatureDataStruct& feature_data, const double time_loop)
{
    if (is_first_run_)
    {
        if (run_params_.debug_flag_ > 1)
        {
            std::cout << "GPModelTypeConcatenatedLOG: Creating gp_initial from training data!\n";
        }
        gp_initial_ = new libgp::GaussianProcess(num_inputs_, gp_params_.covariance_function_params_.final_covfun_);
        hyper_params_.resize(gp_initial_->covf().get_param_dim());
        for (int idx = 0; idx < num_inputs_; idx++)
        {
            hyper_params_(idx) = std::log(gp_params_.covariance_function_params_.ell_t0_(idx));
        }
        hyper_params_(num_inputs_) = log(gp_params_.covariance_function_params_.sf_t0_);
        hyper_params_(num_inputs_ + 1) = log(gp_params_.covariance_function_params_.sn_t0_);
        gp_initial_->covf().set_loghyper(hyper_params_);
        is_first_run_ = false;
    }
    else
    {
        if (time_loop <= gp_params_.max_initial_rec_time_sec_)
        {
            regression_data_model_->prepareRegressionData(feature_data);
            if (buffer_count_++ > gp_params_.input_data_params_.data_history_)
            {
                addPattern(gp_initial_);
            }
            if (run_params_.debug_flag_ > 1 &&
                std::fmod(time_loop - std::floor(time_loop), 1.0) < 1.0 / run_params_.ros_loop_rate_)
            {
                std::cout << "GPModelTypeConcatenatedLOG: Training time for gp_initial: " << (int)time_loop
                          << " (sec)!\n";
            }
        }
        else
        {
            standardizeAndOptimize(gp_initial_);
            switch (utils::checkAndCreateDirectory(gp_params_.filepath_gp_initial_.c_str()))
            {
                case 0:  // Error creating the directory
                {
                    if (run_params_.debug_flag_ >= 0)
                    {
                        std::cout << "\n**GPModelTypeConcatenatedERROR: Error creating the directory: '"
                                  << gp_params_.filepath_gp_initial_.c_str() << "'\n";
                    }
                }
                break;
                case 1:  // Directory already exists.
                {
                    if (run_params_.debug_flag_ > 1)
                    {
                        std::cout << "GPModelTypeConcatenatedLOG: Directory: '"
                                  << gp_params_.filepath_gp_initial_.c_str() << " already exists'\n";
                    }
                }
                break;
                case 2:  // Directory successfully created
                {
                    if (run_params_.debug_flag_ > 1)
                    {
                        std::cout << "GPModelTypeConcatenatedLOG: Creating directory: '"
                                  << gp_params_.filepath_gp_initial_.c_str() << "'\n";
                    }
                }
                break;
            }
            gp_initial_->write(const_cast<char*>(gp_params_.file_gp_initial_.c_str()));
            buffer_count_ = 0;
            is_gp_initial_created_ = true;
        }
    }
}

void GPModelTypeConcatenated::loadInitial()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelTypeConcatenatedLOG: Creating gp_initial from file: "
                  << gp_params_.file_gp_initial_.c_str() << ".\n";
    }
    if (not utils::isFileExist(gp_params_.filepath_gp_initial_) && run_params_.debug_flag_ >= 0)
    {
        std::cout << "\n**GPModelTypeConcatenatedERROR: gp_initial file does not exist at: '"
                  << gp_params_.filepath_gp_initial_.c_str() << "'\n";
        return;
    }
    gp_initial_ = new libgp::GaussianProcess(const_cast<char*>(gp_params_.file_gp_initial_.c_str()));
    if (gp_initial_ == nullptr && run_params_.debug_flag_ >= 0)
    {
        std::cout << "\n**GPModelTypeConcatenatedERROR: Error creating gp_initial. It is a null pointer'\n";
        return;
    }
    extractAndSetStandardizeDataFromGP(gp_initial_);
    is_first_run_ = false;
    is_gp_initial_created_ = true;
}

}  // namespace wind_regression
