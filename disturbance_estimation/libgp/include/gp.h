// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

/*! 
 *  
 *   \page licence Licensing
 *    
 *     libgp - Gaussian process library for Machine Learning
 *
 *      \verbinclude "../COPYING"
 */

#ifndef __GP_H__
#define __GP_H__

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>

#include "cov.h"
#include "sampleset.h"
#define LAMBDA_INITIAL -1             // Added by Mohit
#define LAMBDA_CHANGE_THRESHOLD 1e-2  // Added by Mohit

namespace libgp
{
/** Gaussian process regression.
   *  @author Manuel Blum */
class GaussianProcess
{
public:
    /** Empty initialization */
    GaussianProcess();

    /** Create and instance of GaussianProcess with given input dimensionality 
     *  and covariance function. */
    GaussianProcess(size_t input_dim, std::string covf_def);

    /** Create and instance of GaussianProcess from file. */
    GaussianProcess(const char* filename);

    /** Copy constructor */
    GaussianProcess(const GaussianProcess& gp);

    virtual ~GaussianProcess();

    /** Write current gp model to file. */
    void write(const char* filename);

    /** Predict target value for given input.
     *  @param x input vector
     *  @return predicted value */
    virtual double f(const double x[]);

    // Added by Mohit
    /** Predict target value for given input utilizing forgetting factor approach.
     *  @param x input vector
     *  @param lambda forgetting factor
     *  @return predicted value */
    virtual double f(const double x[], const double& lambda);

    // Added by Mohit
    /** Predict target value for given noisy input.
     *  @param x noisy input vector
     *  @param sx2 input vector noise covariance
     *  @return predicted value */
    virtual double f_noisy_inp(const double x[], Eigen::VectorXd& sx2);

    /** Predict variance of prediction for given input.
     *  @param x input vector
     *  @return predicted variance */
    virtual double var(const double x[]);

    // Added by Mohit
    /** Predict variance of prediction for given input utilizing forgetting factor approach.
     *  @param x input vector
     *  @param lambda forgetting factor
     *  @return predicted variance */
    virtual double var(const double x[], const double& lambda);

    // Added by Mohit
    /** Predict variance of prediction for given noisy input.
     *  @param x noisy input vector
     *  @param mu_star predicted target value for the noisy input vector
     *  @param sx2 input vector noise covariance
     *  @return predicted variance */
    virtual double var_noisy_inp(const double x[], double mu_star, Eigen::VectorXd& sx2);

    // Added by Mohit
    /** Obtain mean and variance together to save computation */
    virtual std::pair<double, double> f_var(const double x[]);

    // Added by Mohit
    /** To compute the paramters for noisy inputs. */
    void update_params_noisy_inp(Eigen::VectorXd& sx2);

    /** Add input-output-pair to sample set.
     *  Add a copy of the given input-output-pair to sample set.
     *  @param x input array
     *  @param y output value
     */
    virtual void add_pattern(const double x[], double y);

    bool set_y(size_t i, double y);

    // Added by Mohit
    /** Standardize the whole sample set values. */
    void standardize_sampleset(Eigen::VectorXd& _mean_x,
                               Eigen::VectorXd& _std_dev_x,
                               double& _mean_y,
                               double& _std_dev_y);

    // Added by Mohit
    /** Standardize the input values only. */
    void standardize_sampleset(Eigen::VectorXd& _mean_x, Eigen::VectorXd& _std_dev_x);

    // Added by Mohit
    /** Standardize the target values only. */
    void standardize_sampleset(double& _mean_y, double& _std_dev_y);

    /** Get number of samples in the training set. */
    size_t get_sampleset_size();

    /** Clear sample set and free memory. */
    void clear_sampleset();

    /** Get reference on currently used covariance function. */
    CovarianceFunction& covf();

    /** Get input vector dimensionality. */
    size_t get_input_dim();

    double log_likelihood();

    Eigen::VectorXd log_likelihood_gradient();

    // Added by Mohit
    /** Get sampleset pointer. */
    SampleSet* get_sampleset();

    // Added by Mohit
    /** Set sampleset pointer. */
    void set_sampleset(SampleSet& _sample_set);

    // Added by Mohit
    /** Get hyperpar_needs_update. */
    bool get_hyperpar_needs_update();

    // Added by Mohit
    /** Remove input-output data at k from the sample set. */
    void remove_sample(size_t k);

protected:
    /** The covariance function of this Gaussian process. */
    CovarianceFunction* cf;

    /** The training sample set. */
    SampleSet* sampleset;

    /** Alpha is cached for performance. */
    Eigen::VectorXd alpha;

    /** Last test kernel vector. */
    Eigen::VectorXd k_star;

    /** Linear solver used to invert the covariance matrix. */
    //    Eigen::LLT<Eigen::MatrixXd> solver;
    Eigen::MatrixXd L;

    // Added by Mohit
    /** Lower triangular covariance matrix computed with forgetting factor. */
    Eigen::MatrixXd L_lambda;

    /** Input vector dimensionality. */
    size_t input_dim;

    /** Update test input and cache kernel vector. */
    void update_k_star(const Eigen::VectorXd& x_star);

    // Added by Mohit
    /** Update test input (with noise) and cache kernel vector. */
    void update_k_star_noisy_inp(const Eigen::VectorXd& x_star, Eigen::VectorXd& sx2);

    virtual void update_alpha();
    // Added by Mohit
    /** Update alpha vector utilizing forgetting factor approach.
     *  @param lambda forgetting factor */
    virtual void update_alpha(const double& lambda);

    /** Compute covariance matrix and perform cholesky decomposition. */
    virtual void compute();

    bool alpha_needs_update;

    // Added by Mohit
    /** Variable holding the standardizing constant for the target values.*/
    double mean_y;
    double std_dev_y;

    // Added by Mohit
    /** Variable holding the standardizing constant for the input values.*/
    Eigen::VectorXd* mean_x;
    Eigen::VectorXd* std_dev_x;

    // Added by Mohit
    /** remove row from an Eigen matrix.*/
    void remove_row(Eigen::MatrixXd& matrix, size_t rowToRemove);
    /** remove column from an Eigen matrix.*/
    void remove_column(Eigen::MatrixXd& matrix, size_t colToRemove);

    // Added by Mohit
    /** Variable holding the previous forgetting factor value.*/
    double lambda_prev = LAMBDA_INITIAL;

    // Added by Mohit
    /** Computes the Gamma matrix for the given forgetting factor.
     *  @param size sampleset size
     *  @param lambda forgetting factor
     *  @return Gamma matrix */
    Eigen::MatrixXd compute_Gamma(const size_t& size, const double& lambda);

    // Added by Mohit
    /** Computes alpha including the given forgetting factor.
     *  @param L lower triangular cholesky decomposition of Ky (i.e. Ky = LL')
     *  @param y target values
     *  @param lambda forgetting factor
     *  @return Gamma matrix */
    Eigen::VectorXd alpha_with_lambda(const Eigen::MatrixXd& L, const Eigen::VectorXd& y, const double& lambda);

private:
    /** No assignement */
    GaussianProcess& operator=(const GaussianProcess&);

    // Added by Mohit
    /** Covariance function hyperparameters **/
    Eigen::VectorXd ell2;
    double sf2;
    double sqrt_det_ell2_by_det_ell2_p_sx2;
    double sqrt_det_ell2_by_det_ell2_p_2sx2;
    bool hyperpar_needs_update;
};
}  // namespace libgp

#endif /* __GP_H__ */
