// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2011, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __GP_SPARSE_FORGET_FACTOR_H__
#define __GP_SPARSE_FORGET_FACTOR_H__

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#define LAMBDA_INITIAL -1
#define LAMBDA_CHANGE_THRESHOLD 1e-2

#include "gp_sparse.h"
#include "gp_forget_factor.h"

namespace libgp
{
/** Sparse Gaussian process regression with forgetting factor.
 *  @author Mohit Mehndiratta */
class SparseGaussianProcessWithForgetFactor : public SparseGaussianProcess, public GaussianProcessWithForgetFactor
{
public:
    /** Create an instance of SparseGaussianProcessWithForgetFactor with given input
      * dimensionality and covariance function. */
    SparseGaussianProcessWithForgetFactor(size_t input_dim, std::string covf_def, size_t _max_points);

    /** Create an instance of SparseGaussianProcessWithForgetFactor from file. */
    SparseGaussianProcessWithForgetFactor(const char* filename);

    /** Create an instance of SparseGaussianProcessWithForgetFactor from Copy constructor */
    SparseGaussianProcessWithForgetFactor(size_t _max_points,
                                          const GaussianProcess& gp,
                                          const double& lambda_initial = LAMBDA_INITIAL);

    virtual ~SparseGaussianProcessWithForgetFactor();

    double var(const double x[]) override;
    double var(const double x[], const double& lambda) override;

protected:
    /** Computes prior mean and variance for induced points utilizing forgetting factor approach.
     *  @param _lambda forgetting factor.  */
    virtual void prior_meanVar_with_lambda(const double& lambda);

    void prior_meanVar(SampleSet* _sampleset) override;
    void update_alpha() override;
    void update_alpha(const double& lambda) override;

private:
    Eigen::MatrixXd Lmm;
    Eigen::MatrixXd Kmu;
    Eigen::VectorXd ym;
};
}  // namespace libgp

#endif /* __GP_FORGET_FACTOR_H__ */
