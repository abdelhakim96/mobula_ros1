// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2011, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __GP_FORGET_FACTOR_H__
#define __GP_FORGET_FACTOR_H__

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#define LAMBDA_INITIAL -1
#define LAMBDA_CHANGE_THRESHOLD 1e-2

#include "gp.h"

namespace libgp
{
/** Gaussian process regression with forgetting factor.
 *  @author Mohit Mehndiratta */
class GaussianProcessWithForgetFactor : public virtual GaussianProcess
{
public:
    /** Create an instance of GaussianProcessWithForgetFactor with given input
     * dimensionality and covariance function. */
    GaussianProcessWithForgetFactor(size_t input_dim, std::string covf_def);

    /** Create an instance of GaussianProcessWithForgetFactor from file. */
    GaussianProcessWithForgetFactor(const char* filename);

    /** Create an instance of GaussianProcessWithForgetFactor from Copy constructor */
    GaussianProcessWithForgetFactor(const GaussianProcess& gp, const double& lambda_initial = LAMBDA_INITIAL);

    virtual ~GaussianProcessWithForgetFactor();

    /** Predict target value for given input utilizing forgetting factor approach.
     *  @param x input vector
     *  @param lambda forgetting factor
     *  @return predicted value */
    virtual double f(const double x[], const double& lambda);

    /** Predict variance of prediction for given input utilizing forgetting factor approach.
     *  @param x input vector
     *  @param lambda forgetting factor
     *  @return predicted variance */
    virtual double var(const double x[], const double& lambda);

    double f(const double x[]) override;
    double var(const double x[]) override;

protected:
    /** Variable holding the previous forgetting factor value.*/
    double lambda_prev = LAMBDA_INITIAL;

    /** Lower triangular covariance matrix computed with forgetting factor. */
    Eigen::MatrixXd L_lambda;

    /** Update alpha vector utilizing forgetting factor approach.
     *  @param lambda forgetting factor */
    virtual void update_alpha(const double& lambda);

    /** Computes the Gamma matrix for the given forgetting factor.
     *  @param size sampleset size
     *  @param lambda forgetting factor
     *  @return Gamma matrix */
    Eigen::MatrixXd compute_Gamma(const size_t& size, const double& lambda);

    /** Computes alpha including the given forgetting factor.
     *  @param L lower triangular cholesky decomposition of Ky (i.e. Ky = LL')
     *  @param y target values
     *  @param lambda forgetting factor
     *  @return Gamma matrix */
    Eigen::VectorXd alpha_with_lambda(const Eigen::MatrixXd& L, const Eigen::VectorXd& y, const double& lambda);

    void update_alpha() override;

private:
};
}  // namespace libgp

#endif /* __GP_FORGET_FACTOR_H__ */
