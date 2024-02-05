// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2011, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __GP_SPARSE_H__
#define __GP_SPARSE_H__

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "gp.h"

namespace libgp
{
/** Sparse Gaussian process regression.
 *  @author Manuel Blum
 *  @author Mohit Mehndiratta */
class SparseGaussianProcess : public GaussianProcess
{
public:
    /** Create an instance of SparseGaussianProcess with given input
     * dimensionality and covariance function. */
    SparseGaussianProcess(size_t input_dim, std::string covf_def, size_t _max_points);

    /** Create an instance of SparseGaussianProcess from file. */
    SparseGaussianProcess(const char* filename);

    /** Create an instance of SparseGaussianProcess from Copy constructor */
    SparseGaussianProcess(size_t _max_points, const GaussianProcess& gp);

    virtual ~SparseGaussianProcess();

    /** Sparcifies the training dataset. */
    virtual void sparsify();

    /** Sparcifies the training dataset by removing the datapoints from particular
     * location.
     * @param i remove_index */
    virtual void sparsify(size_t i);

    /** Computes and gets kernel matrices between samplesets.
     *  @param *sampleset_m pointer to first sampleset
     *  @param *sampleset_u pointer to second sampleset
     *  @return pair of Kmm, and Kmu matrices, wherein Kmm is in lower triangular form. */
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_KmmKmu(SampleSet* sampleset_m, SampleSet* sampleset_u);

    /** Sets maximum number of inducing points.
     *  @param _max_points number of inducing points to be generated */
    void set_max_points(size_t _max_points);

    /** Gets maximum number of computed inducing points.
     *  @return variable max_points */
    size_t get_max_points();

    // Overriding functions
    double var(const double x[]) override;
    std::pair<double, double> f_var(const double x[]) override;

    /** Adds input-output-pair to sampleset and update the SparseGP. If the number of samples is bigger than
     *  the desired maximum points, the SparseGP is re-sparsified and re-computed.
     *  @param x input array
     *  @param y output value */
    void add_pattern(const double x[], double y) override;

protected:
    /** Prior mean for induced points */
    Eigen::VectorXd muu;

    /** Prior covariance matrix for induced points */
    Eigen::MatrixXd Suu;

    void update_alpha() override;

    /** Computes prior mean and variance for induced points
     *  @param _sampleset input sampleset.  */
    virtual void prior_meanVar(SampleSet* _sampleset);

    /** gets the densest point in a list of samples
     *  @param input_dim is the dimensionality of the samples
     *  @param num_samples is the number of samples
     *  @param distance_matrix is an NxN matrix where element (i,j) contains the
     *  (pre)computed distance between the ith and the jth samples
     *  @return gets the index of densest point.  */
    size_t get_most_dense_point(size_t input_dimension, size_t num_samples, const Eigen::MatrixXd& distance_matrix);

private:
    size_t max_points;
};
}  // namespace libgp

#endif /* __GP_SPARSE_H__ */
