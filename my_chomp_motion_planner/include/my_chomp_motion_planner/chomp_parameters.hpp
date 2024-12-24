#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>


namespace chomp
{
/**
 * \class ChompParameters
 * \brief Stores parameters for configuring the CHOMP optimization process.
 *
 * This includes costs (smoothness, obstacle), iteration limits, time limits,
 * and optional parameters like stochastic descent and pseudo-inverse usage.
 */
class ChompParameters
{
public:
  /**
   * \brief Default constructor, initializes all parameters to typical defaults.
   */
  ChompParameters();

  /**
   * \brief Virtual destructor.
   */
  virtual ~ChompParameters();

  /**
   * \brief Adjusts certain parameters (learning rate, ridge factor, time limit, iterations)
   *        as a form of "recovery" if a solution is not found with the nominal values.
   *
   * \param[in] learning_rate         The step size for gradient descent
   * \param[in] ridge_factor          Diagonal regularization added to matrices
   * \param[in] planning_time_limit   Maximum allowed planning time (in seconds)
   * \param[in] max_iterations        Maximum allowed iterations before giving up
   */
  void setRecoveryParams(double learning_rate, double ridge_factor, int planning_time_limit, int max_iterations);

  /**
   * \brief Specify the method to initialize the trajectory before optimization.
   *        Must be one of the strings in VALID_INITIALIZATION_METHODS.
   *
   * \param[in] method  The name of the initialization method (e.g., "linear", "cubic", etc.)
   * \return true if \a method is in VALID_INITIALIZATION_METHODS; false otherwise
   */
  bool setTrajectoryInitializationMethod(std::string method);

  /**
   * \brief Print the parameters into the RCLCPP INFO
   */
  void print(const rclcpp::Logger& logger = rclcpp::get_logger("chomp_parameters")) const;


  // -----------------------------------------------------------
  // Public Members
  // -----------------------------------------------------------

  /**
   * \brief The maximum time (in seconds) the optimizer can run before termination.
   */
  double planning_time_limit_;

  /**
   * \brief The maximum number of iterations for the optimization loop.
   */
  int max_iterations_;

  /**
   * \brief Once a collision-free solution is found, continue for up to this many additional iterations to improve it.
   */
  int max_iterations_after_collision_free_;

  /**
   * \brief Weight factor for the smoothness cost term in the final objective.
   */
  double smoothness_cost_weight_;

  /**
   * \brief Weight factor for the obstacle (collision) cost term in the final objective.
   */
  double obstacle_cost_weight_;

  /**
   * \brief The learning rate used to update the trajectory during optimization.
   */
  double learning_rate_;

  /**
   * \brief Coefficient controlling the velocity component of the smoothness cost.
   */
  double smoothness_cost_velocity_;

  /**
   * \brief Coefficient controlling the acceleration component of the smoothness cost.
   */
  double smoothness_cost_acceleration_;

  /**
   * \brief Coefficient controlling the jerk component of the smoothness cost.
   */
  double smoothness_cost_jerk_;

  /**
   * \brief If true, uses stochastic descent for part of the optimization (sampling random points instead of all).
   */
  bool use_stochastic_descent_;

  /**
   * \brief Diagonal regularization term added to the smoothness cost matrix (prevents singularities).
   */
  double ridge_factor_;

  /**
   * \brief If true, uses pseudo-inverse-based gradient updates (Jacobian pseudo-inverse) instead of Jacobian transpose.
   */
  bool use_pseudo_inverse_;

  /**
   * \brief Additional ridge factor for the pseudo-inverse computation (numerical stability).
   */
  double pseudo_inverse_ridge_factor_;

  /**
   * \brief Maximum step by which a joint can be updated in one iteration (prevents large, destabilizing steps).
   */
  double joint_update_limit_;

  /**
   * \brief Minimum clearance distance (in meters) to maintain from obstacles.
   */
  double min_clearance_;

  /**
   * \brief Collision threshold cost; if below this cost, the trajectory is considered effectively collision-free.
   */
  double collision_threshold_;

  /**
   * \brief If true, the algorithm will run as a simple filter (uses shorter iteration loops, no early exit, etc.).
   */
  bool filter_mode_;

  /**
   * \brief Valid methods for the initial trajectory. Could include "linear", "cubic", "min_jerk", etc.
   */
  static const std::vector<std::string> VALID_INITIALIZATION_METHODS;

  /**
   * \brief Selected method from VALID_INITIALIZATION_METHODS to initialize the trajectory.
   */
  std::string trajectory_initialization_method_;

  /**
   * \brief If true, CHOMP attempts multiple parameter "recoveries" if an initial solution is not found.
   */
  bool enable_failure_recovery_;

  /**
   * \brief Maximum number of recovery attempts to find a collision-free path after initial failure.
   */
  int max_recovery_attempts_;
};

}  // namespace chomp
