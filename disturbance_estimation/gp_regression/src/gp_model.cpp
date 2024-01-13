/**
 * @file   gp_model.cpp
 * @author Mohit Mehndiratta
 * @date   Oct 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

#include <gp_wind_regression/gp_model.h>

namespace wind_regression
{
GPModel::GPModel(const RunParams& run_params, const GPParams& gp_params)
    : run_params_(run_params)
    , gp_model_base_(new GPModelTypeConcatenated(run_params, gp_params))
{
    if (gp_model_base_ == nullptr)
    {
        if (run_params_.debug_flag_ >= 0)
        {
            std::cout << "**GPModelERROR: Created gp_model_base_ is a null pointer.\n";
        }
        // TODO: better error handling condition
        exit(0);
    }
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelLOG: Constructor is called!\n";
    }
}

GPModel::~GPModel()
{
    if (run_params_.debug_flag_ > 1)
    {
        std::cout << "GPModelLOG: Destructor is called!\n";
    }
}

GPModelBase* GPModel::getGPModelBase()
{
    return gp_model_base_;
}

}  // namespace wind_regression
