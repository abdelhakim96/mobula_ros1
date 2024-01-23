/**
 * @file   gp_model_base.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/gp_model_base.h>

namespace wind_regression
{
GPModelBase::GPModelBase(const RunParams& run_params, const GPParams& gp_params)
    : run_params_(run_params)
    , gp_params_(gp_params)
    , regression_data_model_(new RegressionDataModel(run_params, gp_params))
{
    num_inputs_ = gp_params.input_data_params_.num_inputs_;

    if (regression_data_model_ == nullptr)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**GPModelBaseERROR: Created regression_data_model_ is a null pointer.\n";
        }
        // TODO: better error handling condition
        exit(0);
    }

    // Initialize prediction_data_;
    prediction_data_.mean_.resize(gp_params.num_predict_points_future_, gp_params.mean_t0_);
    prediction_data_.mean_unstandardized_ = prediction_data_.mean_;
    prediction_data_.variance_.resize(gp_params.num_predict_points_future_, gp_params.variance_t0_);
    prediction_data_.variance_unstandardized_ = prediction_data_.variance_;
    prediction_data_.mean_p_2std_dev_.resize(gp_params.num_predict_points_future_,
                                             gp_params.mean_t0_ + 2 * std::sqrt(gp_params.variance_t0_));
    prediction_data_.mean_m_2std_dev_.resize(gp_params.num_predict_points_future_,
                                             gp_params.mean_t0_ - 2 * std::sqrt(gp_params.variance_t0_));

    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelBaseLOG: Constructor is called!\n";
    }
}

GPModelBase::GPModelBase()
{
    fprintf(stderr, "This (GPModelBase::GPModelBase()) should never be called");
}

GPModelBase::~GPModelBase()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelBaseLOG: Destructor is called!\n";
    }
}

FeatureDataStruct GPModelBase::getStandardizedFeatureData(const FeatureDataStruct& feature_data)
{
    FeatureDataStruct standardize_feature_data;
    standardize_feature_data.setZeros(feature_data.current_time.input.size());
    // Standardize inputs
    for (int idx = 0; idx < feature_data.current_time.input.size(); idx++)
    {
        standardize_feature_data.previous_time.input[idx] =
            (feature_data.previous_time.input[idx] - standardize_data_.mean_inputs_[idx]) /
            standardize_data_.std_dev_inputs_[idx];
        standardize_feature_data.current_time.input[idx] =
            (feature_data.current_time.input[idx] - standardize_data_.mean_inputs_[idx]) /
            standardize_data_.std_dev_inputs_[idx];
    }
    // Standardize targets
    standardize_feature_data.previous_time.target =
        (feature_data.previous_time.target - standardize_data_.mean_targets_) / standardize_data_.std_dev_targets_;
    standardize_feature_data.current_time.target =
        (feature_data.current_time.target - standardize_data_.mean_targets_) / standardize_data_.std_dev_targets_;
    return standardize_feature_data;
}

void GPModelBase::addPattern(libgp::GaussianProcess*& gp, const RegressionDataStruct& regression_data)
{
    // Adds training patterns
    gp->add_pattern(utils::getarrayFromStdVector(regression_data.train.input), regression_data.train.target);
}

void GPModelBase::addPattern(libgp::GaussianProcess*& gp)
{
    addPattern(gp, regression_data_model_->getRegressionData());
}

void GPModelBase::addPatternwithRemoval(libgp::GaussianProcess*& gp, const size_t& idx)
{
    // Always remove data from the starting location of the moving window
    //    _gp->get_sampleset()->remove(idx);
    gp->remove_sample(idx);

    addPattern(gp);
}

void GPModelBase::computeMeanVariance(libgp::GaussianProcess*& gp,
                                      const double& mean_0,
                                      const size_t& idx,
                                      double points[],
                                      double lamb)
{
    prediction_data_.mean_[idx] = mean_0 + gp->f(points,lamb);
    prediction_data_.variance_[idx] = gp->var(points);
    computeUnstandardizedPredictionData(idx);
}

void GPModelBase::computeMeanVarianceNoisyInputs(libgp::GaussianProcess*& gp,
                                                 const double& mean_0,
                                                 const size_t& idx,
                                                 double points[])
{
    prediction_data_.mean_[idx] = mean_0 + gp->f_noisy_inp(points, gp_params_.covariance_function_params_.sx_t0_);
    //    if (idx == gp_params_.num_predict_points_future_ - 1)
    //    {
    //        prediction_data_.variance_[idx] =
    //            gp_->var_noisy_inp(points, prediction_data_.mean_[idx], gp_params_.inputs_noise_variance_);
    //        //        std::cout<<"prediction_data_.variance_["<<idx<<"] = "<<prediction_data_.variance_[idx]<<"\n";
    //    }
    //    else
    prediction_data_.variance_[idx] = gp->var(points);
    computeUnstandardizedPredictionData(idx);
}

void GPModelBase::computeUnstandardizedPredictionData(const size_t& idx)
{
    prediction_data_.mean_unstandardized_[idx] =
        prediction_data_.mean_[idx] * standardize_data_.std_dev_targets_ + standardize_data_.mean_targets_;
    prediction_data_.variance_unstandardized_[idx] =
        prediction_data_.variance_[idx] * standardize_data_.std_dev_targets_ + standardize_data_.mean_targets_;
    prediction_data_.mean_p_2std_dev_[idx] =
        prediction_data_.mean_unstandardized_[idx] + 2 * sqrt(prediction_data_.variance_unstandardized_[idx]);
    prediction_data_.mean_m_2std_dev_[idx] =
        prediction_data_.mean_unstandardized_[idx] - 2 * sqrt(prediction_data_.variance_unstandardized_[idx]);
}

void GPModelBase::standardizeAndOptimize(libgp::GaussianProcess*& gp)
{
    // Standardization of the training data
    // TODO: select the most applicable one

    // Gets standardize_data struct.
    gp->standardize_sampleset(standardize_data_.mean_inputs_,
                              standardize_data_.std_dev_inputs_,
                              standardize_data_.mean_targets_,
                              standardize_data_.std_dev_targets_);

    switch (gp_params_.hyper_optimize_method_)
    {
        case HyperOptimizeMethod::CG:
        {
            if (run_params_.debug_flag_ > 1)
            {
                std::cout << "GPModelBaseLOG: Optimizing hyper parameters using CG method. \n";
            }
            libgp::CG cg;
            cg.maximize(*&gp, gp_params_.max_gradient_updates_, 0);
        }
        break;
        case HyperOptimizeMethod::RPROP:
        {
            if (run_params_.debug_flag_ > 1)
            {
                std::cout << "GPModelBaseLOG: Optimizing hyper parameters using RPROP method. \n";
            }
            libgp::CG rprop;
            rprop.maximize(*&gp, gp_params_.max_gradient_updates_, 0);
        }
        break;
        default:
        {
            if (run_params_.debug_flag_ >= 1)
            {
                std::cout << "**GPModelBaseWARN: Optimizing method not found. Optimizing hyper parameters using CG "
                             "method. \n";
            }
            libgp::CG cg;
            cg.maximize(*&gp, gp_params_.max_gradient_updates_, 0);
        }
        break;
    }

    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelBaseLOG: Optimized hyperparams = ";
        for (int idx = 0; idx < hyper_params_.size(); idx++)
        {
            std::cout << exp(gp->covf().get_loghyper()(idx));
            if (idx < hyper_params_.size() - 1)
            {
                std::cout << ", ";
            }
            else
            {
                std::cout << "\n";
            }
        }
    }
}

void GPModelBase::extractAndSetStandardizeDataFromGP(libgp::GaussianProcess*& gp)
{
    standardize_data_.mean_inputs_ = gp->get_sampleset()->mean_x();
    standardize_data_.std_dev_inputs_ = gp->get_sampleset()->std_dev_x();
    standardize_data_.mean_targets_ = gp->get_sampleset()->mean_y();
    standardize_data_.std_dev_targets_ = gp->get_sampleset()->std_dev_y();
}

bool GPModelBase::isRegressionInitialized()
{
    return is_regression_initialized_;
}

PredictionStruct GPModelBase::getPredictionData()
{
    return prediction_data_;
}

StandardizeStruct GPModelBase::getStandardizeData()
{
    return standardize_data_;
}
}  // namespace wind_regression
