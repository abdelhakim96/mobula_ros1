// libgp - Gaussian process library for Machine Learning
// Copyright (c) 2013, Manuel Blum <mblum@informatik.uni-freiburg.de>
// All rights reserved.

// Modified by Mohit Mehndiratta <mohit005@e.ntu.edu.sg> 2019

#include "gp_sparse.h"

namespace libgp
{
SparseGaussianProcess::SparseGaussianProcess(size_t input_dim, std::string covf_def, size_t _max_points)
    : GaussianProcess(input_dim, covf_def)
    , max_points(_max_points)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcess is called!\n";
#endif
}

SparseGaussianProcess::SparseGaussianProcess(const char* filename)
    : GaussianProcess(filename)
{
    max_points = this->sampleset->size();
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcess is called!\n";
#endif
}

SparseGaussianProcess::SparseGaussianProcess(size_t _max_points, const GaussianProcess& gp)
    : GaussianProcess(gp)
    , max_points(_max_points)
{
#if DEBUG_MODE
    std::cout << "Parameter Constructor of SparseGaussianProcess is called!\n";
#endif
}

SparseGaussianProcess::~SparseGaussianProcess()
{
#if DEBUG_MODE
    std::cout << "Destructor of SparseGaussianProcess is called!\n";
#endif
}

void SparseGaussianProcess::sparsify()
{
    SampleSet* sampleset_induced_points = new SampleSet(*sampleset);

    /// Computes a bigger size distance matrix to avoid re-computations
    Eigen::MatrixXd distance_matrix = Eigen::MatrixXd::Zero(sampleset->size(), sampleset->size());
    for (size_t row_idx = 0; row_idx < sampleset->size(); ++row_idx)
    {
        for (size_t col_idx = 0; col_idx < sampleset->size(); ++col_idx)
        {
            distance_matrix(row_idx, col_idx) = (sampleset->x(row_idx) - sampleset->x(col_idx)).norm();
        }
    }

    while (sampleset_induced_points->size() > max_points)
    {
        uint dense_point_idx = get_most_dense_point(input_dim, sampleset_induced_points->size(), distance_matrix);
        /// sanity check
        if (dense_point_idx < 0)
        {
            break;
        }
        sampleset_induced_points->remove(dense_point_idx);

        remove_column(distance_matrix, dense_point_idx);
        remove_row(distance_matrix, dense_point_idx);
    }

    // compute prior mean and covariance for induced points
    prior_meanVar(sampleset_induced_points);

    // replace measurements with induced points
    delete sampleset;
    sampleset = sampleset_induced_points;

    cf->loghyper_changed = true;
}

void SparseGaussianProcess::sparsify(size_t i)
{
    SampleSet* sampleset_induced_points = new SampleSet(*sampleset);

    sampleset_induced_points->remove(i);

    // compute prior mean and covariance for induced points
    prior_meanVar(sampleset_induced_points);

    // replace measurements with induced points
    delete sampleset;
    sampleset = sampleset_induced_points;

    cf->loghyper_changed = true;
}

size_t SparseGaussianProcess::get_most_dense_point(size_t input_dim,
                                                   size_t num_samples,
                                                   const Eigen::MatrixXd& distance_matrix)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t dense_point_idx = -1;

    for (size_t idx = 0; idx < num_samples; ++idx)
    {
        double distance = 0;
        std::vector<double> neighbors(num_samples);
        Eigen::VectorXd::Map(neighbors.data(), neighbors.size()) = distance_matrix.row(idx);
        /// remove self distance
        neighbors.erase(neighbors.begin() + idx);

        std::partial_sort(neighbors.begin(), neighbors.begin() + input_dim, neighbors.end());

        for (size_t j = 0; j < input_dim; j++)
        {
            distance += neighbors[j];
        }
        if (distance < min_distance)
        {
            min_distance = distance;
            dense_point_idx = idx;
        }
    }
    return dense_point_idx;
}

void SparseGaussianProcess::prior_meanVar(SampleSet* _sampleset)
{
    size_t num_dense_points = sampleset->size();
    size_t num_induced_points = _sampleset->size();

    Eigen::MatrixXd Kmm(num_dense_points, num_dense_points), Kmu(num_dense_points, num_induced_points);
    std::tie(Kmm, Kmu) = compute_KmmKmu(sampleset, _sampleset);

    // muu = Kmu'*inv(Kmm + Sfm)*fmh = Kmu'*inv(LL')*fmh
    muu.resize(num_induced_points);
    // The following utilizes precomputed value of alpha that is based on dense sampleset. As such, L is inherited from the base gp class.
    muu = Kmu.transpose() * alpha;
    // alpha to be updated again with muu for prediction
    alpha_needs_update = true;

    Eigen::MatrixXd Q(num_induced_points, num_induced_points);
    // Q = Kmu'inv(Kmm + Sfm)*Kmu = Kmu'inv(LL')*Kmu
    Q = Kmu.transpose() * (Kmm.selfadjointView<Eigen::Lower>().llt().solve(Kmu));

    Suu.setZero(num_induced_points, num_induced_points);
    // Note that Suu = Kuu - Q.topLeftCorner(n, n) but we purposely set Suu = Q, to be helpful later
    Suu = Q;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> SparseGaussianProcess::compute_KmmKmu(SampleSet* sampleset_m,
                                                                                  SampleSet* sampleset_u)
{
    size_t num_m_points = sampleset_m->size();
    size_t num_u_points = sampleset_u->size();

    // gets kernel matrix of dense sampleset (lower triangle) Kmm = LL'
    Eigen::MatrixXd Kmm(num_m_points, num_m_points);
    Kmm.setZero(num_m_points, num_m_points);
    Kmm = (L.topLeftCorner(num_m_points, num_m_points) * L.topLeftCorner(num_m_points, num_m_points).transpose())
              .triangularView<Eigen::Lower>();

    // compute kernel matrix
    Eigen::MatrixXd Kmu(num_m_points, num_u_points);
    Kmu.setZero(num_m_points, num_u_points);
    for (size_t row = 0; row < num_m_points; ++row)
    {
        for (size_t col = 0; col < num_u_points; ++col)
        {
            // TODO: check if the values for noise covariance is coming up in the
            // below evaluation -> yes, it should be coming!
            Kmu(row, col) = cf->get(sampleset_m->x(row), sampleset_u->x(col));
        }
    }

    return std::make_pair(Kmm, Kmu);
}

void SparseGaussianProcess::update_alpha()
{
    // can previously computed values be used?
    if (!alpha_needs_update)
        return;
    alpha_needs_update = false;
    alpha.resize(sampleset->size());
    int n = sampleset->size();

    // alpha = inv(L)*muu, where Kuu = LL'
    alpha = L.topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(muu);
    // alpha = inv(L')*alpha
    L.topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(
        alpha);  // // Same result with using transpose instead of adjoint. Using adjoint for consistency with libgp
}

double SparseGaussianProcess::var(const double x[])
{
    if (sampleset->empty())
        return 0;
    Eigen::Map<const Eigen::VectorXd> x_star(x, input_dim);
    // Note that the sampleset has been updated to represent inducing points rather than dense points.
    compute();  // This computes L (i.e. Kuu = LL')
    update_alpha();
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

std::pair<double, double> SparseGaussianProcess::f_var(const double x[])
{
    if (sampleset->empty())
        return std::make_pair(0, 0);

    return std::make_pair(f(x), var(x));
}

void SparseGaussianProcess::add_pattern(const double x[], double y)
{
    this->GaussianProcess::add_pattern(x, y);

    // if we surpassed the maximum points, re-sparsify and recompute
    if (sampleset->size() > max_points)
    {
        sparsify();
    }
}

size_t SparseGaussianProcess::get_max_points()
{
    return max_points;
}

void SparseGaussianProcess::set_max_points(size_t _max_points)
{
    max_points = _max_points;
}

}  // namespace libgp
