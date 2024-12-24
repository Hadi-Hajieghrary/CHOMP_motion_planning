#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <random>
#include <cstdlib>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Core>

namespace chomp
{

/**
 * \brief The length of the finite-difference rule used in CHOMP derivative computations.
 * 
 * Typically, this means we have a 7-point stencil centered on the current index:
 * i - 3, i - 2, i - 1, i, i + 1, i + 2, i + 3
 */
static const int DIFF_RULE_LENGTH = 7;

/**
 * \brief Pre-defined finite-difference rules for velocity, acceleration, and jerk
 *
 * Each row of DIFF_RULES corresponds to a derivative order:
 *  - 0: velocity
 *  - 1: acceleration
 *  - 2: jerk
 * 
 * Each rule is an array of length DIFF_RULE_LENGTH (7), centered around the middle.
 */
static const double DIFF_RULES[3][DIFF_RULE_LENGTH] = {
  // Velocity rule:
  //   { 0, 0, -2/6, -3/6, 6/6, -1/6, 0 }
  //   The nonzero coefficients approximate first-order derivative at the center.
  { 0, 0, -2.0 / 6.0, -3.0 / 6.0, 6.0 / 6.0, -1.0 / 6.0, 0 },

  // Acceleration rule:
  //   { 0, -1/12, 16/12, -30/12, 16/12, -1/12, 0 }
  //   Approximates the second derivative.
  { 0, -1.0 / 12.0, 16.0 / 12.0, -30.0 / 12.0, 16.0 / 12.0, -1.0 / 12.0, 0 },

  // Jerk rule:
  //   { 0, 1/12, -17/12, 46/12, -46/12, 17/12, -1/12 }
  //   Approximates the third derivative.
  { 0, 1.0 / 12.0, -17.0 / 12.0, 46.0 / 12.0, -46.0 / 12.0, 17.0 / 12.0, -1.0 / 12.0 }
};

/**
 * \brief Copies the joint positions from a RobotState into a row of an Eigen::MatrixXd
 *
 * This function extracts the active joints in the specified \a planning_group_name
 * and writes their positions into \a joint_array. The \a joint_array is assumed to be
 * sized correctly (or at least have enough columns) to hold these positions.
 *
 * \param[in] state                The RobotState containing current joint positions
 * \param[in] planning_group_name  The name of the JointModelGroup to extract
 * \param[out] joint_array         The row expression in Eigen to fill with the joint values
 */
static inline void robotStateToArray(const moveit::core::RobotState& state,
                                     const std::string& planning_group_name,
                                     Eigen::MatrixXd::RowXpr joint_array)
{
  // Retrieve the group from the RobotState
  const moveit::core::JointModelGroup* group = state.getJointModelGroup(planning_group_name);

  // Keep track of current index for writing joint values
  size_t joint_index = 0;

  // For each active joint model in the group, set the corresponding array element
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
  {
    // getFirstVariableIndex() is the index of the first dimension of this joint in the state's variable array
    joint_array[joint_index++] = state.getVariablePosition(jm->getFirstVariableIndex());
  }
}

/**
 * \brief Normalize an angle into the range [0, 2*pi)
 *
 * This function first reduces the angle using fmod(angle, 2*pi), then ensures positivity
 * by adding 2*pi if the result is negative, and finally takes another mod with 2*pi.
 *
 * \param[in] angle  The input angle (in radians)
 * \return           The normalized angle in [0, 2*pi)
 */
static inline double normalizeAnglePositive(double angle)
{
  // fmod() handles floating-point remainder
  return std::fmod(std::fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

/**
 * \brief Normalize an angle into the range (-pi, pi]
 *
 * This function first normalizes the angle into [0, 2*pi) with normalizeAnglePositive(),
 * then if the result is greater than pi, subtracts 2*pi to bring it into the (-pi, pi] range.
 *
 * \param[in] angle  The input angle (in radians)
 * \return           The normalized angle in (-pi, pi]
 */
static inline double normalizeAngle(double angle)
{
  double a = normalizeAnglePositive(angle);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

/**
 * \brief Compute the shortest angular distance from \a start to \a end
 *
 * This function determines how far, and in which direction, we should travel from
 * \a start to \a end in the minimal sense, accounting for wrap-around at ±pi.
 *
 * \param[in] start  Starting angle (radians)
 * \param[in] end    Target angle (radians)
 * \return           The signed smallest difference in [(-pi, pi], in radians
 */
static inline double shortestAngularDistance(double start, double end)
{
  // Normalize both angles to [0, 2*pi) range, compute the difference, then re-normalize
  double res = normalizeAnglePositive(normalizeAnglePositive(end) - normalizeAnglePositive(start));

  // If the difference is larger than pi, adjust by -(2*pi - difference)
  // to get the negative equivalent rotation.
  if (res > M_PI)
  {
    res = -(2.0 * M_PI - res);
  }

  // Finally, bring result into (-pi, pi] range
  return normalizeAngle(res);
}


/**
 * \class MultivariateGaussian
 * \brief Generates samples from a multivariate Gaussian (normal) distribution.
 *
 * Usage example:
 * \code
 *   Eigen::VectorXd mean(3);
 *   mean << 0.0, 1.0, 2.0;
 *   Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(3, 3);
 *
 *   chomp::MultivariateGaussian sampler(mean, cov);
 *
 *   Eigen::VectorXd sample(3);
 *   sampler.sample(sample);
 *   // 'sample' now contains a single random draw from N(mean, cov)
 * \endcode
 */
class MultivariateGaussian
{
public:
  /**
   * \brief Constructs a multivariate Gaussian from a given mean vector and covariance matrix.
   *
   * Internally performs a Cholesky decomposition of the \a covariance to facilitate fast sampling.
   *
   * \tparam Derived1   Type of the mean vector (e.g., Eigen::VectorXd)
   * \tparam Derived2   Type of the covariance matrix (e.g., Eigen::MatrixXd)
   * \param mean        The mean vector of the distribution
   * \param covariance  The covariance matrix of the distribution (must be symmetric & positive-definite)
   */
  template <typename Derived1, typename Derived2>
  MultivariateGaussian(const Eigen::MatrixBase<Derived1>& mean, const Eigen::MatrixBase<Derived2>& covariance);

  /**
   * \brief Draws a single random sample from the distribution and stores it in \a output
   *
   * The function internally samples from a standard normal distribution and then
   * transforms it according to the Cholesky factor of the covariance.
   *
   * \tparam Derived        Type of the output vector (e.g., Eigen::VectorXd)
   * \param[out] output     The vector in which the sampled data is stored
   */
  template <typename Derived>
  void sample(Eigen::MatrixBase<Derived>& output);

private:
  Eigen::VectorXd mean_;                /**< Mean vector of the Gaussian distribution */
  Eigen::MatrixXd covariance_;          /**< Covariance matrix of the Gaussian distribution */
  Eigen::MatrixXd covariance_cholesky_; /**< Cholesky factor (L) of covariance_, where covariance_ = LL^T */

  int size_; /**< Dimension of the distribution (i.e., rows in mean_) */

  std::mt19937 rng_;                  /**< Mersenne Twister random number generator */
  std::normal_distribution<double> gaussian_; /**< Standard normal distribution generator (0 mean, unit variance) */
};

////////////////////////////////////////////////////////////////////////////////
// Template function definitions
////////////////////////////////////////////////////////////////////////////////

/**
 * \brief Constructor Implementation
 */
template <typename Derived1, typename Derived2>
MultivariateGaussian::MultivariateGaussian(const Eigen::MatrixBase<Derived1>& mean,
                                           const Eigen::MatrixBase<Derived2>& covariance)
  : mean_(mean)
  , covariance_(covariance)
  , covariance_cholesky_(covariance_.llt().matrixL())  // Cholesky factor L
  , gaussian_(0.0, 1.0)                                // Standard normal (mean=0, stddev=1)
{
  // Seed the random generator with a non-deterministic source
  rng_ = std::mt19937(std::random_device{}());
  // The distribution dimension is simply the number of rows of 'mean'
  size_ = mean.rows();
}

/**
 * \brief Sample Implementation
 */
template <typename Derived>
void MultivariateGaussian::sample(Eigen::MatrixBase<Derived>& output)
{
  // First fill 'output' with standard normal samples
  for (int i = 0; i < size_; ++i)
  {
    output(i) = gaussian_(rng_);
  }

  // Transform by L (Cholesky factor) and shift by the mean:
  //    sample = mean + L * z, where z ~ N(0, I)
  output = mean_ + covariance_cholesky_ * output;
}


}  // namespace chomp
