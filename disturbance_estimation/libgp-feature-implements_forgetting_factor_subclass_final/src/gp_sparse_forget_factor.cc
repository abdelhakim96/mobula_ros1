// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

// Modified by Mohit Mehndiratta <mohit005@e.ntu.edu.sg> 2024

#include "gp_sparse_forget_factor.h"

namespace libgp
{
SparseGaussianProcessWithForgetFactor::SparseGaussianProcessWithForgetFactor(size_t input_dim,
                                                                             std::string covf_def,
                                                                             size_t _max_points)
    : GaussianProcess(input_dim, covf_def)
    , SparseGaussianProcess(input_dim, covf_def, _max_points)
    , GaussianProcessWithForgetFactor(input_dim, covf_def)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcessWithForgetFactor is called!\n";
#endif
}

SparseGaussianProcessWithForgetFactor::SparseGaussianProcessWithForgetFactor(const char* filename)
    : GaussianProcess(filename)
    , SparseGaussianProcess(filename)
    , GaussianProcessWithForgetFactor(filename)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcessWithForgetFactor is called!\n";
#endif
}

SparseGaussianProcessWithForgetFactor::SparseGaussianProcessWithForgetFactor(size_t _max_points,
                                                                             const GaussianProcess& gp,
                                                                             const double& lambda_initial)
    : GaussianProcess(gp)
    , SparseGaussianProcess(_max_points, gp)
    , GaussianProcessWithForgetFactor(gp, lambda_initial)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcessWithForgetFactor is called!\n";
#endif
}

SparseGaussianProcessWithForgetFactor::~SparseGaussianProcessWithForgetFactor()
{
#if DEBUG_MODE
    std::cout << "Destructor of SparseGaussianProcessWithForgetFactor is called!\n";
#endif
}

double SparseGaussianProcessWithForgetFactor::var(const double x[])
{
    // calling the base class's function
    return this->GaussianProcessWithForgetFactor::var(x);
}

double SparseGaussianProcessWithForgetFactor::var(const double x[], const double& lambda)
{
    if (sampleset->empty())
        return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    // Note that the sampleset has been updated to represent inducing points rather than dense points.
    compute();  // This computes L (i.e. Kuu = LL')
    update_alpha(lambda);
    update_k_star(x_star);  // This computes k_star (i.e., k_star_u)

    int n = sampleset->size();
    // v = inv(L)*k_star
    Eigen::VectorXd v = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star);
    // v = inv(L')*v
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(
        v);  // Same result with using adjoint instead of transpose.
    // var_f_star = k_star_star - v'*(Kuu - Suu)*v = k_star_star - v'*(Q)*v
    return cf->get(x_star, x_star) - v.dot(Suu * v);
}

void SparseGaussianProcessWithForgetFactor::prior_meanVar(SampleSet* _sampleset)
{
    // calling the base class's function
    SparseGaussianProcess::prior_meanVar(_sampleset);

    size_t num_dense_points = sampleset->size();
    size_t num_induced_points = _sampleset->size();

    // storing variables for later use before the sampleset gets updated
    Lmm.resize(num_dense_points, num_dense_points);
    Kmu.resize(num_dense_points, num_induced_points);
    ym.resize(num_dense_points);

    // gets Lmm from the previously saved variable
    Lmm = L.topLeftCorner(num_dense_points, num_dense_points);
    // gets Kmu from the function call
    Kmu = (compute_KmmKmu(sampleset, _sampleset)).second;
    // Map target values to VectorXd
    const std::vector<double>& targets = sampleset->y();
    Eigen::Map<const Eigen::VectorXd> y(&targets[0], num_dense_points);
    this->ym = y;
}

void SparseGaussianProcessWithForgetFactor::prior_meanVar_with_lambda(const double& lambda)
{
    // can previously computed values be used?
    if (std::abs(lambda_prev - lambda) <= LAMBDA_CHANGE_THRESHOLD)
        return;

    size_t num_induced_points = sampleset->size();

    // compute alpha including the effect of forgetting factor
    Eigen::VectorXd alpha_lambda = alpha_with_lambda(Lmm, ym, lambda);

    // muu = Kmu'*inv(Kmm + Sfm)*fmh = Kmu'*inv(LL')*fmh
    muu = Kmu.transpose() * alpha_lambda;
    // alpha to be updated again with muu for prediction
    alpha_needs_update = true;

    Eigen::MatrixXd Q(num_induced_points, num_induced_points);
    // gets Kmm with the forgetting factor
    Eigen::MatrixXd Kmm = (L_lambda * L_lambda.transpose()).triangularView<Eigen::Lower>();
    // Q = Kmu'inv(Kmm + Sfm)*Kmu = Kmu'inv(LL')*Kmu
    Q = Kmu.transpose() * (Kmm.selfadjointView<Eigen::Lower>().llt().solve(Kmu));

    Suu.setZero(num_induced_points, num_induced_points);
    // Note that Suu = Kuu - Q.topLeftCorner(n, n) but we purposely set Suu = Q, to be helpful later
    Suu = Q;

    lambda_prev = lambda;
}

void SparseGaussianProcessWithForgetFactor::update_alpha()
{
    // check if forgetting factor approach has been used
    // This approach resets the lambda_prev, which makes
    //it recompute alpha when the forgetting factor function is called.
    //This causes one extra computation.
    // TODO: find a better approach.
    if (lambda_prev != LAMBDA_INITIAL)
    {
        alpha_needs_update = true;
        lambda_prev = LAMBDA_INITIAL;
    }

    // finally, update alpha
    SparseGaussianProcess::update_alpha();
}

void SparseGaussianProcessWithForgetFactor::update_alpha(const double& lambda)
{
    // update prior mean and variance for the given forgetting factor value
    prior_meanVar_with_lambda(lambda);

    // finally, update alpha
    update_alpha();
}

}  // namespace libgp
