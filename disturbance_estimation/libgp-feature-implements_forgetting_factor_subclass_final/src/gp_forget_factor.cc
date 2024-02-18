// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

// Modified by Mohit Mehndiratta <mohit005@e.ntu.edu.sg> 2024

#include "gp_forget_factor.h"

namespace libgp
{
GaussianProcessWithForgetFactor::GaussianProcessWithForgetFactor(size_t input_dim, std::string covf_def)
    : GaussianProcess(input_dim, covf_def)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of GaussianProcessWithForgetFactor is called!\n";
#endif
}

GaussianProcessWithForgetFactor::GaussianProcessWithForgetFactor(const char* filename)
    : GaussianProcess(filename)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of GaussianProcessWithForgetFactor is called!\n";
#endif
}

GaussianProcessWithForgetFactor::GaussianProcessWithForgetFactor(const GaussianProcess& gp,
                                                                 const double& lambda_initial)
    : GaussianProcess(gp)
    , lambda_prev(lambda_initial)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of GaussianProcessWithForgetFactor is called!\n";
#endif
}

GaussianProcessWithForgetFactor::~GaussianProcessWithForgetFactor()
{
#if DEBUG_MODE
    std::cout << "Destructor of GaussianProcessWithForgetFactor is called!\n";
#endif
}

double GaussianProcessWithForgetFactor::f(const double x[])
{
    // calling the base class's function
    return this->GaussianProcess::f(x);
}

double GaussianProcessWithForgetFactor::f(const double x[], const double& lambda)
{
    if (sampleset->empty())
        return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha(lambda);
    update_k_star(x_star);
    // expected_f_star = k_star'*alpha
    return k_star.dot(alpha);
}

double GaussianProcessWithForgetFactor::var(const double x[])
{
    // calling the base class's function
    return GaussianProcess::var(x);
}

double GaussianProcessWithForgetFactor::var(const double x[], const double& lambda)
{
    if (sampleset->empty())
        return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    compute();
    update_alpha(lambda);
    update_k_star(x_star);
    // v = inv(L_lambda)*k_star
    Eigen::VectorXd v = L_lambda.triangularView<Eigen::Lower>().solve(k_star);
    // var_f_star = k_star_star - v'*v
    return cf->get(x_star, x_star) - v.dot(v);
}

void GaussianProcessWithForgetFactor::update_alpha()
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
    GaussianProcess::update_alpha();
}

void GaussianProcessWithForgetFactor::update_alpha(const double& lambda)
{
    // can previously computed values be used?
    if (!alpha_needs_update && std::abs(lambda_prev - lambda) <= LAMBDA_CHANGE_THRESHOLD)
        return;
    alpha_needs_update = false;
    alpha.resize(sampleset->size());
    // Map target values to VectorXd
    const std::vector<double>& targets = sampleset->y();
    Eigen::Map<const Eigen::VectorXd> y(&targets[0], sampleset->size());
    int n = sampleset->size();
    // compute alpha including the effect of forgetting factor
    alpha = alpha_with_lambda(L.topLeftCorner(n, n), y, lambda);
    lambda_prev = lambda;
}

Eigen::MatrixXd GaussianProcessWithForgetFactor::compute_Gamma(const size_t& size, const double& lambda)
{
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Identity(size, size);
    for (int i = size - 2; i >= 0; i--)
    {
        Gamma(i, i) = Gamma(i + 1, i + 1) * lambda;
    }
    return Gamma;
}

Eigen::VectorXd GaussianProcessWithForgetFactor::alpha_with_lambda(const Eigen::MatrixXd& L,
                                                                   const Eigen::VectorXd& y,
                                                                   const double& lambda)
{
    size_t size = L.rows();

    // Given Ky = LL'; Gamma = GG'
    // compute the mean as: m = k * inv(L * Gamma * L') * Gamma * y
    //                      m = k * inv(G' * L') * inv(L * G) * Gamma * y
    // Compute forgetting factor matrix
    Eigen::MatrixXd Gamma = compute_Gamma(size, lambda);
    // target values updated with forgetting factor
    Eigen::VectorXd y_lambda = Gamma * y;

    // Note that we are reusing/updating variable Gamma for later usage
    // perform cholesky factorization: Gamma = chol(Gamma)
    Gamma = Gamma.selfadjointView<Eigen::Lower>().llt().matrixL();
    // L_lambda = L * G;
    L_lambda = L.triangularView<Eigen::Lower>() * Gamma;

    // alpha_lambda = inv(L_lambda * G)*y_lambda
    Eigen::VectorXd alpha_lambda = L_lambda.triangularView<Eigen::Lower>().solve(y_lambda);
    // alpha_lambda = inv(G' * L_lambda')*alpha_lambda
    L_lambda.triangularView<Eigen::Lower>().adjoint().solveInPlace(alpha_lambda);

    return alpha_lambda;
}

}  // namespace libgp
