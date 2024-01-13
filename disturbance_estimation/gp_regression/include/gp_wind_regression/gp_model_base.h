/**
 * @file   gp_model_base.h
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief
 */

#ifndef GP_MODEL_BASE_H
#define GP_MODEL_BASE_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports
#include <Eigen/Dense>
#include "gp.h"
#include "gp_sparse.h"
#include "gp_utils.h"
#include "rprop.h"
#include "cg.h"

/// Internal imports
#include <gp_wind_regression/regression_data_model.h>
#include <gp_wind_regression/launch_parameters.h>
#include <gp_wind_regression/data_struct.h>
#include <gp_wind_regression/utils.h>

namespace wind_regression
{
struct StandardizeStruct
{
    Eigen::VectorXd mean_inputs_;
    Eigen::VectorXd std_dev_inputs_;
    double mean_targets_;
    double std_dev_targets_;

    void printStandardizeData()
    {
        std::cout << "-----------------------------------\n";
        std::cout << "mean_inputs = " << mean_inputs_ << "\n";
        std::cout << "std_dev_inputs = " << std_dev_inputs_ << "\n";
        std::cout << "mean_targets = " << mean_targets_ << "\n";
        std::cout << "std_dev_targets = " << std_dev_targets_ << "\n";
        std::cout << "-----------------------------------\n";
    }
};

struct PredictionStruct
{
    std::vector<double> mean_;
    std::vector<double> mean_unstandardized_;
    std::vector<double> variance_;
    std::vector<double> variance_unstandardized_;
    std::vector<double> mean_p_2std_dev_;
    std::vector<double> mean_m_2std_dev_;
};

class GPModelBase
{
    //private:
protected:
    RunParams run_params_;
    GPParams gp_params_;

    bool is_regression_initialized_ = false;
    size_t num_inputs_;

    //    RegressionStruct regression_data_;
    /// Instance of the feature data model class
    RegressionDataModel* regression_data_model_;
    StandardizeStruct standardize_data_;
    PredictionStruct prediction_data_;

    libgp::GaussianProcess* gp_;
    Eigen::VectorXd hyper_params_;

public:
    /// Constructor
    /**
     * @brief
     */
    GPModelBase();

    /**
     * @brief
     */
    GPModelBase(const RunParams& run_params, const GPParams& gp_params);

    /// Destructor
    /**
     * @brief
     */
    ~GPModelBase();

    /// Public Member Methods
    /**
     * @brief
     */
    virtual void initialize(const FeatureDataStruct& feature_data, const double time_loop)
    {
    }

    /**
     * @brief
     */
    virtual void core(const FeatureDataStruct& feature_data)
    {
    }

    /**
     * @brief
     */
    FeatureDataStruct getStandardizedFeatureData(const FeatureDataStruct& feature_data);

    /**
     * @brief
     */
    void addPattern(libgp::GaussianProcess*& gp, const RegressionDataStruct& regression_data);

    /**
     * @brief
     */
    void addPattern(libgp::GaussianProcess*& gp);

    /**
     * @brief
     */
    void addPatternwithRemoval(libgp::GaussianProcess*& gp, const size_t& idx);

    /**
     * @brief
     */
    void computeMeanVariance(libgp::GaussianProcess*& gp, const double& mean_0, const size_t& idx, double points[]);

    /**
     * @brief
     */
    void computeMeanVarianceNoisyInputs(libgp::GaussianProcess*& gp,
                                        const double& mean_0,
                                        const size_t& idx,
                                        double points[]);

    /**
     * @brief
     */
    void computeUnstandardizedPredictionData(const size_t& idx);

    /**
     * @brief
     */
    void standardizeAndOptimize(libgp::GaussianProcess*& gp);

    /**
     * @brief
     */
    void extractAndSetStandardizeDataFromGP(libgp::GaussianProcess*& gp);

    /**
     * @brief
     */
    bool isRegressionInitialized();

    /**
     * @brief
     */
    PredictionStruct getPredictionData();

    /**
     * @brief
     */
    StandardizeStruct getStandardizeData();
};

}  // end namespace wind_regression

#endif
