#include <my_chomp_motion_planner/chomp_parameters.hpp>
#include <algorithm>
#include <utility>  // for std::move

namespace chomp
{
/**
 * \class ChompParameters
 * \brief Stores the parameters that configure CHOMP's optimization.
 *
 * This version includes some alternative default parameter values 
 * (e.g., planning_time_limit_ = 6.0, max_iterations_ = 50, etc.) 
 * compared to another implementation.
 *
 * Key parameters include:
 *  - \a smoothness_cost_weight_ to scale smoothness cost
 *  - \a obstacle_cost_weight_ to scale collision cost
 *  - \a learning_rate_ to govern step size
 *  - \a min_clearance_ to define how close we can get to obstacles
 *  - \a collision_threshold_ to decide when we are collision-free enough
 *  - \a use_stochastic_descent_ to pick random points for collision checks
 *  - and more...
 */
ChompParameters::ChompParameters()
{
  // Time limit (seconds) before optimization will terminate
  planning_time_limit_ = 6.0;

  // Main iteration limit
  max_iterations_ = 50;

  // Number of extra iterations after first collision-free solution is found
  max_iterations_after_collision_free_ = 5;

  // Weight given to smoothness cost
  smoothness_cost_weight_ = 0.1;

  // Weight given to collision/obstacle cost
  obstacle_cost_weight_ = 1.0;

  // Learning rate for gradient updates
  learning_rate_ = 0.01;

  // Smoothness cost coefficients for velocity, acceleration, jerk
  smoothness_cost_velocity_ = 0.0;
  smoothness_cost_acceleration_ = 1.0;
  smoothness_cost_jerk_ = 0.0;

  // Ridge factor for diagonal regularization in the cost matrix
  ridge_factor_ = 0.0;

  // Whether to use a pseudo-inverse approach (Jacobian pseudoinverse)
  use_pseudo_inverse_ = false;

  // Additional diagonal ridge when computing the pseudo-inverse
  pseudo_inverse_ridge_factor_ = 1e-4;

  // The maximum step by which each joint can move in one iteration
  joint_update_limit_ = 0.1;

  // Minimum clearance (m) to obstacles
  min_clearance_ = 0.2;

  // Collision threshold cost; if the actual cost is below this, it's considered collision-free
  collision_threshold_ = 0.07;

  // If true, picks random trajectory points to compute collision gradients (stochastic descent)
  use_stochastic_descent_ = true;

  // If true, this parameter set is used in a filter mode with different iteration logic
  filter_mode_ = false;

  // Default trajectory initialization method
  trajectory_initialization_method_ = std::string("quintic-spline");

  // Whether we allow attempts to relax or vary the parameters after a failure
  enable_failure_recovery_ = false;

  // How many times to retry if we fail initially
  max_recovery_attempts_ = 5;
}

/**
 * \brief Virtual destructor
 */
ChompParameters::~ChompParameters() = default;

/**
 * \brief Update parameters used in a "recovery" scenario 
 *        (if a solution was not found with nominal parameters).
 *
 * \param[in] learning_rate      Updated learning rate
 * \param[in] ridge_factor       Updated ridge factor
 * \param[in] planning_time_limit Updated time limit (seconds)
 * \param[in] max_iterations     Updated iteration limit
 */
void ChompParameters::setRecoveryParams(double learning_rate, double ridge_factor, int planning_time_limit,
                                        int max_iterations)
{
  this->learning_rate_       = learning_rate;
  this->ridge_factor_        = ridge_factor;
  this->planning_time_limit_ = static_cast<double>(planning_time_limit);
  this->max_iterations_      = max_iterations;
}

/**
 * \brief Lists valid methods for trajectory initialization:
 *   - quintic-spline
 *   - linear
 *   - cubic
 *   - fillTrajectory
 */
const std::vector<std::string> ChompParameters::VALID_INITIALIZATION_METHODS{
  "quintic-spline", "linear", "cubic", "fillTrajectory"
};

/**
 * \brief Sets the method used to initialize the trajectory. 
 *        Must be one of VALID_INITIALIZATION_METHODS.
 *
 * \param[in] method  Name of the initialization method
 * \return true if \a method is recognized as valid, false otherwise
 */
bool ChompParameters::setTrajectoryInitializationMethod(std::string method)
{
  // Check if the provided method is in the list of valid methods
  if (std::find(VALID_INITIALIZATION_METHODS.cbegin(), VALID_INITIALIZATION_METHODS.cend(), method) !=
      VALID_INITIALIZATION_METHODS.end())
  {
    // Move the string to avoid extra copies
    this->trajectory_initialization_method_ = std::move(method);
    return true;
  }
  return false;
}

/**
 * \brief Print the parameters into the RCLCPP INFO
 */
void  ChompParameters::print(const rclcpp::Logger& logger) const
{
  RCLCPP_INFO(logger, "=== CHOMP Parameters ===");
  RCLCPP_INFO(logger, "planning_time_limit_: %f", planning_time_limit_);
  RCLCPP_INFO(logger, "max_iterations_: %d", max_iterations_);
  RCLCPP_INFO(logger, "max_iterations_after_collision_free_: %d", max_iterations_after_collision_free_);
  RCLCPP_INFO(logger, "smoothness_cost_weight_: %f", smoothness_cost_weight_);
  RCLCPP_INFO(logger, "obstacle_cost_weight_: %f", obstacle_cost_weight_);
  RCLCPP_INFO(logger, "learning_rate_: %f", learning_rate_);
  RCLCPP_INFO(logger, "smoothness_cost_velocity_: %f", smoothness_cost_velocity_);
  RCLCPP_INFO(logger, "smoothness_cost_acceleration_: %f", smoothness_cost_acceleration_);
  RCLCPP_INFO(logger, "smoothness_cost_jerk_: %f", smoothness_cost_jerk_);
  RCLCPP_INFO(logger, "use_stochastic_descent_: %s", use_stochastic_descent_ ? "true" : "false");
  RCLCPP_INFO(logger, "ridge_factor_: %f", ridge_factor_);
  RCLCPP_INFO(logger, "use_pseudo_inverse_: %s", use_pseudo_inverse_ ? "true" : "false");
  RCLCPP_INFO(logger, "pseudo_inverse_ridge_factor_: %f", pseudo_inverse_ridge_factor_);
  RCLCPP_INFO(logger, "joint_update_limit_: %f", joint_update_limit_);
  RCLCPP_INFO(logger, "min_clearance_: %f", min_clearance_);
  RCLCPP_INFO(logger, "collision_threshold_: %f", collision_threshold_);
  RCLCPP_INFO(logger, "filter_mode_: %s", filter_mode_ ? "true" : "false");
  RCLCPP_INFO(logger, "trajectory_initialization_method_: %s", trajectory_initialization_method_.c_str());
  RCLCPP_INFO(logger, "enable_failure_recovery_: %s", enable_failure_recovery_ ? "true" : "false");
  RCLCPP_INFO(logger, "max_recovery_attempts_: %d", max_recovery_attempts_);
  RCLCPP_INFO(logger, "=========================");
}



}  // namespace chomp
