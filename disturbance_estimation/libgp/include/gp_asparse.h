// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2011, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

#ifndef __AGP_SPARSE_H__
#define __AGP_SPARSE_H__

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "gp.h"

namespace libgp {
  
  /** Sparse Gaussian process regression.
   *  @author Manuel Blum 
   *  Modified by Mohit Mehndiratta */
  class AdaptiveSparseGaussianProcess : public GaussianProcess
  {
  public:
    
    /** Create an instance of AdaptiveSparseGaussianProcess with given input dimensionality and covariance function. */
    AdaptiveSparseGaussianProcess (size_t input_dim, std::string covf_def, size_t _max_points);
    
    /** Create an instance of AdaptiveSparseGaussianProcess from file. */
    AdaptiveSparseGaussianProcess (const char * filename);
    
    /** Create an instance of AdaptiveSparseGaussianProcess from Copy constructor */
    AdaptiveSparseGaussianProcess (size_t _max_points, const GaussianProcess& gp);

    virtual ~AdaptiveSparseGaussianProcess ();
    
    /** to compute necessary matrices */
//    virtual void initialize(Eigen::VectorXd &_mu_star, Eigen::VectorXd &_sx_star2);

    virtual void sparsify();

    /** Predict variance of prediction for given input.
     *  @param x input vector
     *  @return predicted variance */
    virtual double var(const double x[]);

    /** Obtain mean and variance together to save computation */
    virtual std::pair<double, double> f_var(const double x[]);



    virtual double f(const double x[], double lamb);

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
      compute_KmmKuuKmu(SampleSet * sampleset_m, SampleSet * sampleset_u);

//    double expected_post_var(SampleSet *& _sampleset);

    virtual void add_pattern(const double x[], double y);

    size_t get_max_points();
    void set_max_points(size_t _max_points);

  protected:

    /** Prior mean for induced points */
    Eigen::VectorXd muu;
    
    /** Prior covariance matrix for induced points */
    Eigen::MatrixXd Suu;

    virtual void update_alpha();

    virtual void prior_meanVar(SampleSet * _sampleset);

    double minimize_fun(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);

    /** get the densest point in a list of samples
     * D is the dimensionality of the samples
     * N is the number of samples
     * distances is an NxN matrix where element (i,j) contains
         the (pre)computed distance between the ith and the jth samples */
    int get_most_dense_point(size_t D, size_t N, const Eigen::MatrixXd& distances);

    void remove_row(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
    void remove_column(Eigen::MatrixXd& matrix, unsigned int colToRemove);

  private:
    
    size_t max_points;

    /** Covariance function hyperparameters **/
    Eigen::VectorXd mu_star;
    Eigen::VectorXd sx_star2;
    Eigen::VectorXd ell2;
    double sf2;
    double sqrt_det_ell2_by_det_ell2_p_2sx2;

  };
}

#endif /* __AGP_SPARSE_H__ */
