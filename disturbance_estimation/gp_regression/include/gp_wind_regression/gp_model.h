/**
 * @file   gp_model.h
 * @author Mohit Mehndiratta
 * @date   Sep 2021
 *
 * @copyright
 * Copyright (C) 2021.
 */

/**
 * @brief
 */

#ifndef GP_MODEL_H
#define GP_MODEL_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// Bad headers with problem go here
// restore compiler switches
#pragma GCC diagnostic pop

/// External imports

/// Internal imports
#include <gp_wind_regression/launch_parameters.h>
#include <gp_wind_regression/gp_model_base.h>
#include <gp_wind_regression/gp_model_type_concatenated.h>

namespace wind_regression
{
class GPModel
{
private:
    RunParams run_params_;
    GPModelBase* gp_model_base_;

public:
    /// Constructor
    /**
     * @brief
     */
    GPModel();

    /**
     * @brief
     */
    GPModel(const RunParams& run_params, const GPParams& gp_params);

    /// Destructor
    /**
     * @brief
     */
    ~GPModel();

    /// Public Member Methods
    /**
     * @brief
     */
    GPModelBase* getGPModelBase();
};

}  // end namespace wind_regression

#endif
