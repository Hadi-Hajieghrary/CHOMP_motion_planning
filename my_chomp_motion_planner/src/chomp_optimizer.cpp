#include "my_chomp_motion_planner/chomp_optimizer.hpp"
#include "my_chomp_motion_planner/chomp_utils.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <random>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cfloat>   // for DBL_MAX
#include <chrono>   // for timing


static const rclcpp::Logger LOGGER = rclcpp::get_logger("chomp_optimizer");


namespace chomp
{

ChompCost::ChompCost(const ChompTrajectory& trajectory,
                     const std::vector<double>& derivative_costs,
                     double ridge_factor)
{
  // Total number of trajectory points (including boundary points)
  int num_vars_all = trajectory.getNumPoints();
  // Number of free variables (excluding boundary points)
  // DIFF_RULE_LENGTH - 1 boundary points on each end are not free
  int num_vars_free = num_vars_all - 2 * (DIFF_RULE_LENGTH - 1);
  // Allocate the NxN matrices to zero
  // quad_cost_full_ is used for the entire (boundary + free) domain
  quad_cost_full_ = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
  // We will reuse this for each derivative order
  Eigen::MatrixXd diff_matrix = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
  // 1) Construct the quad cost for all variables as a sum of squared differentiation matrices:
  //    derivative_costs[i] * (D_i.transpose() * D_i),
  //    where D_i is the finite-difference matrix for derivative order i.
  // 2) Multiply by discretization each time, so the 1st derivative cost is proportional to
  //    dt, 2nd derivative cost is proportional to dt^2, etc.
  double multiplier = 1.0;

  for (std::size_t i = 0; i < derivative_costs.size(); ++i)
  {
    double cost_value = derivative_costs[i];
    
    // Multiply by the time discretization each iteration
    multiplier *= trajectory.getDiscretization();
    
    // Build the finite-difference matrix for this derivative order using the global DIFF_RULES array
    // (For example, DIFF_RULES[i] might be {1, -2, 1} for a 2nd derivative.)
    // We pass the pointer to the first element of DIFF_RULES[i].
    diff_matrix = getDiffMatrix(num_vars_all, &DIFF_RULES[i][0]);
    
    // Accumulate into quad_cost_full_:
    //   cost_value * multiplier * (diff_matrix^T * diff_matrix)
    quad_cost_full_ += (cost_value * multiplier) * (diff_matrix.transpose() * diff_matrix);
  }

  // Add a ridge factor for numerical stability.
  // This ensures the matrix is not singular (helps with inversion).
  quad_cost_full_.diagonal().array() += ridge_factor;

  // Extract the sub-block of quad_cost_full_ that corresponds to the free variables
  // (excludes the boundary points).
  quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, 
                                      DIFF_RULE_LENGTH - 1, 
                                      num_vars_free, 
                                      num_vars_free);
  // Invert the sub-block to get quad_cost_inv_
  quad_cost_inv_ = quad_cost_.inverse();
}

Eigen::MatrixXd ChompCost::getDiffMatrix(int size, const double* diff_rule) const
{
  // Build an (size x size) matrix where each row i copies the finite-difference stencil 
  // around index i. Indices outside [0, size-1] are skipped.
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);

  // Typically, DIFF_RULE_LENGTH might be 3, 5, 7, etc. depending on the smoothing order.
  // We center the stencil around i, i.e. for j in [-k, ..., +k].
  // For example, if DIFF_RULE_LENGTH=3, j goes from -1 to +1.
  for (int i = 0; i < size; ++i)
  {
    for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; ++j)
    {
      int index = i + j;

      // Skip if the index is out of the matrix bounds
      if (index < 0 || index >= size)
        continue;

      // Copy the rule value from diff_rule:
      // offset = j + (DIFF_RULE_LENGTH / 2) transforms j into [0, DIFF_RULE_LENGTH-1]
      matrix(i, index) = diff_rule[j + (DIFF_RULE_LENGTH / 2)];
    }
  }

  return matrix;
}

double ChompCost::getMaxQuadCostInvValue() const
{
  // Fetch the maximum coefficient in quad_cost_inv_.
  // maxCoeff() is an Eigen function that returns the maximum element in the entire matrix.
  return quad_cost_inv_.maxCoeff();
}

void ChompCost::scale(double scale)
{
  // The approach is to multiply quad_cost_ and quad_cost_full_ by 'scale',
  // but multiply quad_cost_inv_ by the inverse of 'scale'.
  double inv_scale = 1.0 / scale;

  // Scale the inverse matrix in the opposite direction
  quad_cost_inv_ *= inv_scale;

  // Scale the free-variables matrix
  quad_cost_ *= scale;

  // Scale the full matrix
  quad_cost_full_ *= scale;
}

ChompCost::~ChompCost() = default;



static const rclcpp::Logger LOGGER = rclcpp::get_logger("chomp_optimizer");

/**
 * \brief Helper function for generating a random double between [0,1).
 * \return A uniform random double in [0,1).
 */
double getRandomDouble()
{
  std::default_random_engine seed;
  std::uniform_real_distribution<> uniform(0.0, 1.0);
  return uniform(seed);
}

ChompOptimizer::ChompOptimizer(ChompTrajectory* trajectory,
                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const std::string& planning_group,
                               const ChompParameters* parameters,
                               const moveit::core::RobotState& start_state)
  : full_trajectory_(trajectory)
  , robot_model_(planning_scene->getRobotModel())
  , planning_group_(planning_group)
  , parameters_(parameters)
  , group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH)
  , planning_scene_(planning_scene)
  , state_(start_state)
  , start_state_(start_state)
  , initialized_(false)
{
  RCLCPP_INFO(LOGGER, "Active collision detector is: %s", planning_scene->getCollisionDetectorName().c_str());

  // Attempt to retrieve a collision environment specialized in distance fields
  const collision_detection::CollisionEnvConstPtr& collision_env = planning_scene->getCollisionEnv(planning_scene->getCollisionDetectorName());
  hy_env_ = dynamic_cast<const collision_detection::CollisionEnvHybrid*>(collision_env.get());

  if (!hy_env_)
  {
    RCLCPP_WARN(LOGGER, "Could not initialize hybrid collision world from planning scene");
    return;
  }

  // Finish setup by calling initialization routine
  initialize();
}

/**
 * Perform the core initialization of the optimizer, setting up collision gradients,
 * smoothness costs, and memory buffers.
 */
void ChompOptimizer::initialize()
{
  //--------------------------------------------------------------------------
  // Step 1: Set basic dimensioning values
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 1: Setting dimensioning values...");
  num_vars_free_    = group_trajectory_.getNumFreePoints();
  num_vars_all_     = group_trajectory_.getNumPoints();
  num_joints_       = group_trajectory_.getNumJoints();
  free_vars_start_  = group_trajectory_.getStartIndex();
  free_vars_end_    = group_trajectory_.getEndIndex();
  RCLCPP_INFO(LOGGER, "  - num_vars_free_: %d, num_vars_all_: %d, num_joints_: %d, "
                      "free_vars_start_: %d, free_vars_end_: %d",
              num_vars_free_, num_vars_all_, num_joints_, free_vars_start_, free_vars_end_);

  //--------------------------------------------------------------------------
  // Step 2: Measure collision gradient time
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 2: Measuring collision gradients...");
  auto collision_start_time = std::chrono::system_clock::now();
  collision_detection::CollisionRequest req;
  req.group_name = planning_group_;
  collision_detection::CollisionResult res;

  try
  {
    hy_env_->getCollisionGradients(req, res, state_, &planning_scene_->getAllowedCollisionMatrix(), gsr_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Exception during collision gradient computation: %s", e.what());
    return;
  }

  double collision_duration =
      std::chrono::duration<double>(std::chrono::system_clock::now() - collision_start_time).count();
  RCLCPP_INFO(LOGGER, "  - Collision check duration: %f seconds", collision_duration);

  //--------------------------------------------------------------------------
  // Step 3: Count collision points using STL
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 3: Counting collision points...");
  num_collision_points_ = std::accumulate(
      gsr_->gradients_.begin(), gsr_->gradients_.end(), 0,
      [](int sum, const collision_detection::GradientInfo& gradient) {
        return sum + gradient.gradients.size();
      });
  RCLCPP_INFO(LOGGER, "  - Number of collision points: %d", num_collision_points_);

  //--------------------------------------------------------------------------
  // Step 4: Initialize smoothness costs
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 4: Initializing smoothness costs...");
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;

  joint_model_group_ = planning_scene_->getRobotModel()->getJointModelGroup(planning_group_);
  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < joint_models.size(); ++i)
  {
    double joint_cost = 1.0;
    std::vector<double> derivative_costs(3);
    derivative_costs[0] = joint_cost * parameters_->smoothness_cost_velocity_;
    derivative_costs[1] = joint_cost * parameters_->smoothness_cost_acceleration_;
    derivative_costs[2] = joint_cost * parameters_->smoothness_cost_jerk_;
    joint_costs_.push_back(ChompCost(group_trajectory_, derivative_costs, parameters_->ridge_factor_));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }
  RCLCPP_INFO(LOGGER, "  - Computed max_cost_scale: %f", max_cost_scale);

  //--------------------------------------------------------------------------
  // Step 5: Scale all costs
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 5: Scaling all costs...");
  std::for_each(joint_costs_.begin(), joint_costs_.end(), [=](auto& cost) {
    cost.scale(max_cost_scale);
  });

  //--------------------------------------------------------------------------
  // Step 6: Initialize memory-efficient containers
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 6: Initializing memory-efficient containers...");

  auto initialize_matrix = [&](auto& matrix, int rows, int cols) {
    matrix = Eigen::MatrixXd::Zero(rows, cols);
  };

  initialize_matrix(smoothness_increments_, num_vars_free_, num_joints_);
  initialize_matrix(collision_increments_, num_vars_free_, num_joints_);
  initialize_matrix(final_increments_, num_vars_free_, num_joints_);

  smoothness_derivative_         = Eigen::VectorXd::Zero(num_vars_all_);
  jacobian_                      = Eigen::MatrixXd::Zero(3, num_joints_);
  jacobian_pseudo_inverse_       = Eigen::MatrixXd::Zero(num_joints_, 3);
  jacobian_jacobian_tranpose_    = Eigen::MatrixXd::Zero(3, 3);
  random_state_                  = Eigen::VectorXd::Zero(num_joints_);
  joint_state_velocities_        = Eigen::VectorXd::Zero(num_joints_);

  //--------------------------------------------------------------------------
  // Step 7: Set up trajectory backups
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 7: Setting up trajectory backups...");
  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_   = group_trajectory_.getTrajectory();

  //--------------------------------------------------------------------------
  // Step 8: Resize auxiliary containers
  //--------------------------------------------------------------------------
  collision_point_joint_names_.resize(num_vars_all_, std::vector<std::string>(num_collision_points_));
  collision_point_pos_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  collision_point_vel_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  collision_point_acc_eigen_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  joint_axes_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));
  joint_positions_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_joints_));
  collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_potential_gradient_.resize(num_vars_all_, EigenSTL::vector_Vector3d(num_collision_points_));
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));

  //--------------------------------------------------------------------------
  // Step 9: Handle fixed joints and build mappings
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 9: Handling fixed joints and building mappings...");
  std::map<std::string, std::string> fixed_link_resolution_map;
  std::transform(joint_models.begin(), joint_models.end(), std::back_inserter(joint_names_),
                 [](const auto* joint_model) {
                   return joint_model->getName();
                 });
  for (const auto* joint : joint_model_group_->getFixedJointModels())
  {
    if (joint->getParentLinkModel())
    {
      fixed_link_resolution_map[joint->getName()] =
          joint->getParentLinkModel()->getParentJointModel()->getName();
    }
  }
  for (const auto* link : joint_model_group_->getUpdatedLinkModels())
  {
    if (fixed_link_resolution_map.find(link->getParentJointModel()->getName()) == fixed_link_resolution_map.end())
    {
      const moveit::core::JointModel* parent_model = link->getParentJointModel();
      while (parent_model->getParentLinkModel())
      {
        parent_model = parent_model->getParentLinkModel()->getParentJointModel();
        if (std::find(joint_names_.begin(), joint_names_.end(), parent_model->getName()) != joint_names_.end())
        {
          break;
        }
      }
      fixed_link_resolution_map[link->getParentJointModel()->getName()] = parent_model->getName();
    }
  }

  //--------------------------------------------------------------------------
  // Step 10: Set collision point data
  //--------------------------------------------------------------------------
  RCLCPP_INFO(LOGGER, "Step 10: Setting collision point data...");
  for (int i = free_vars_start_; i <= free_vars_end_; ++i)
  {
    size_t j = 0;
    for (const auto& info : gsr_->gradients_)
    {
      std::transform(info.sphere_locations.begin(),
                     info.sphere_locations.end(),
                     collision_point_joint_names_[i].begin() + j,
                     [&](const auto&) {
                       return fixed_link_resolution_map[info.joint_name];
                     });
      j += info.sphere_locations.size();
    }
  }

  //--------------------------------------------------------------------------
  // Finalize
  //--------------------------------------------------------------------------
  initialized_ = true;
  RCLCPP_INFO(LOGGER, "Initialization complete.");
}



ChompOptimizer::~ChompOptimizer()
{
  destroy();
}

void ChompOptimizer::registerParents(const moveit::core::JointModel* model)
{
  // If the model is the root joint, we are done
  if (model == robot_model_->getRootJoint())
    return;

  const moveit::core::JointModel* parent_model = nullptr;
  bool found_root = false;

  // Ascend the chain until we reach the root
  while (!found_root)
  {
    if (parent_model == nullptr)
    {
      if (!model->getParentLinkModel() || !model->getParentLinkModel()->getParentJointModel())
      {
        RCLCPP_ERROR(LOGGER, "Model %s is not root but has a null parent link or joint!", model->getName().c_str());
        return;
      }
      parent_model = model->getParentLinkModel()->getParentJointModel();
    }
    else
    {
      if (parent_model == robot_model_->getRootJoint())
      {
        found_root = true;
      }
      else
      {
        parent_model = parent_model->getParentLinkModel()->getParentJointModel();
      }
    }
    // Mark the parent as true
    joint_parent_map_[model->getName()][parent_model->getName()] = true;
  }
}


std::pair<double, double> ChompOptimizer::stepOptimization(){

  // Update the collision info by forward kinematics
  performForwardKinematics();

  // Evaluate collision cost and smoothness cost
  double c_cost = getCollisionCost();
  double s_cost = getSmoothnessCost();
  double cost   = c_cost + s_cost;
  RCLCPP_INFO(LOGGER, "Collision cost %f, smoothness cost: %f", c_cost, s_cost);

  // Track best solution so far
  if (iteration_ == 0)
  {
    best_group_trajectory_      = group_trajectory_.getTrajectory();
    best_group_trajectory_cost_ = cost;
    last_improvement_iteration_ = iteration_;
  }
  else if (cost < best_group_trajectory_cost_)
  {
    best_group_trajectory_      = group_trajectory_.getTrajectory();
    best_group_trajectory_cost_ = cost;
    last_improvement_iteration_ = iteration_;
  }

  // Compute incremental updates
  calculateSmoothnessIncrements();
  calculateCollisionIncrements();
  calculateTotalIncrements();

  // If not using HMC, just do a gradient update
  addIncrementsToTrajectory();

  // Respect joint limits
  handleJointLimits();

  // Copy local trajectory to the full_trajectory_
  updateFullTrajectory();

  return {c_cost, s_cost};
}


bool ChompOptimizer::optimize(std::vector<std::pair<ChompTrajectory, double>>& intermediate_results) { 
  
  intermediate_results.clear(); 
  bool optimization_result = false;  // Return value
  auto start_time = std::chrono::system_clock::now();

  bool should_break_out = false;
  RCLCPP_INFO(LOGGER, "Maximum Number of Iterations: %d", parameters_->max_iterations_);
  for (iteration_ = 0; iteration_ < parameters_->max_iterations_; ++iteration_)
  {
    auto [intermediate_c_cost, intermediate_s_cost] = stepOptimization();
    // Add the current trajectory and cost to intermediate results
    intermediate_results.emplace_back(group_trajectory_, intermediate_c_cost + intermediate_s_cost);
 
    // The initial trajectory is assumed to be an optimal one - based on the fill method chosen
    // therefore, if it is a viable trajectory, e.g., collision-free, we may break out from the
    // optimization rule.
    if (iteration_ % 10 == 0)
    {
      RCLCPP_INFO(LOGGER, "iteration: %d", iteration_);
      if (isCurrentTrajectoryMeshToMeshCollisionFree())
      {
        num_collision_free_iterations_ = 0;
        RCLCPP_INFO(LOGGER, "Chomp got mesh-to-mesh safety at iter %d. Breaking out early.", iteration_);
        is_collision_free_ = true;
        iteration_++;
        should_break_out = true;
      }
    }

    // If not in filter mode, we break out when collision cost is sufficiently small
    if (!parameters_->filter_mode_)
    {
      if (intermediate_c_cost < parameters_->collision_threshold_)
      {
        num_collision_free_iterations_ = parameters_->max_iterations_after_collision_free_;
        is_collision_free_ = true;
        iteration_++;
        should_break_out = true;
      }
      else
      {
        RCLCPP_INFO(LOGGER, "The Collision Cost %f is over threshold %f", intermediate_c_cost, parameters_->collision_threshold_);
      }
    }

    // If time is exceeded, break
    if (std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count() >
        parameters_->planning_time_limit_)
    {
      RCLCPP_WARN(LOGGER, "Breaking out early due to time limit constraints.");
      break;
    }

    // If we just discovered a collision-free path
    if (should_break_out)
    {
      collision_free_iteration_++;
      if (num_collision_free_iterations_ == 0)
      {
        break;
      }
      else if (collision_free_iteration_ > num_collision_free_iterations_)
      {
        break;
      }
    }
  }

  return optimization_result;
}



bool ChompOptimizer::isCurrentTrajectoryMeshToMeshCollisionFree() const
{
  moveit_msgs::msg::RobotTrajectory traj;
  traj.joint_trajectory.joint_names = joint_names_;

  for (size_t i = 0; i < group_trajectory_.getNumPoints(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (size_t j = 0; j < group_trajectory_.getNumJoints(); ++j)
    {
      point.positions.push_back(best_group_trajectory_(i, j));
    }
    traj.joint_trajectory.points.push_back(point);
  }
  moveit_msgs::msg::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(start_state_, start_state_msg);
  return planning_scene_->isPathValid(start_state_msg, traj, planning_group_);
}

/**
 * Calculate smoothness increments for each joint using STL algorithms.
 */
void ChompOptimizer::calculateSmoothnessIncrements()
{
    std::for_each(joint_costs_.begin(), joint_costs_.end(), [this, idx = 0](const auto& joint_cost) mutable {
        joint_cost.getDerivative(group_trajectory_.getJointTrajectory(idx), smoothness_derivative_);
        smoothness_increments_.col(idx++) = -smoothness_derivative_.segment(group_trajectory_.getStartIndex(), num_vars_free_);
    });
}

void ChompOptimizer::calculateCollisionIncrements()
{
  collision_increments_.setZero(num_vars_free_, num_joints_);

  // We can do partial updates (stochastic) or full updates
  int start_point = 0;
  int end_point   = free_vars_end_;

  if (parameters_->use_stochastic_descent_)
  {
    start_point = static_cast<int>(getRandomDouble() * (free_vars_end_ - free_vars_start_)) + free_vars_start_;
    if (start_point < free_vars_start_)
      start_point = free_vars_start_;
    if (start_point > free_vars_end_)
      start_point = free_vars_end_;
    end_point = start_point;
  }
  else
  {
    start_point = free_vars_start_;
  }

  for (int i = start_point; i <= end_point; ++i)
  {
    for (int j = 0; j < num_collision_points_; ++j)
    {
      double potential = collision_point_potential_[i][j];
      if (potential < 1e-4)
        continue;

      // The negative gradient of potential
      Eigen::Vector3d potential_gradient = -collision_point_potential_gradient_[i][j];

      double vel_mag     = collision_point_vel_mag_[i][j];
      double vel_mag_sq  = vel_mag * vel_mag;

      // (From CHOMP paper) We define:
      //   normalized_velocity = v / |v|
      //   projector = I - vv^T / |v|^2
      //   curvature = projector * accel / |v|^2
      //
      // cartesian_gradient = |v| * [projector * potential_gradient - potential * curvature]
      Eigen::Vector3d normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      Eigen::Matrix3d orthogonal_projector =
          Eigen::Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());

      Eigen::Vector3d curvature_vector =
          (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;

      Eigen::Vector3d cartesian_gradient = vel_mag *
          (orthogonal_projector * potential_gradient - potential * curvature_vector);

      // Pass cartesian_gradient through the Jacobian transpose
      getJacobian(i, collision_point_pos_eigen_[i][j],
                  collision_point_joint_names_[i][j], jacobian_);

      if (parameters_->use_pseudo_inverse_)
      {
        calculatePseudoInverse();
        collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_pseudo_inverse_ * cartesian_gradient;
      }
      else
      {
        collision_increments_.row(i - free_vars_start_).transpose() -= jacobian_.transpose() * cartesian_gradient;
      }
    }
  }
}

void ChompOptimizer::calculatePseudoInverse()
{
  // Basic method: pseudo-inverse = J^T * (J*J^T + ridge*I)^-1
  jacobian_jacobian_tranpose_ =
      jacobian_ * jacobian_.transpose()
      + Eigen::MatrixXd::Identity(3, 3) * parameters_->pseudo_inverse_ridge_factor_;

  jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void ChompOptimizer::calculateTotalIncrements()
{
  // final_increments = alpha * (Q^-1 [ lambda_s * smoothness_increments + lambda_c * collision_increments ])
  for (int i = 0; i < num_joints_; ++i)
  {
    final_increments_.col(i) =
        parameters_->learning_rate_ *
        (joint_costs_[i].getQuadraticCostInverse() *
         (parameters_->smoothness_cost_weight_ * smoothness_increments_.col(i) +
          parameters_->obstacle_cost_weight_   * collision_increments_.col(i)));
  }
}

void ChompOptimizer::addIncrementsToTrajectory()
{
  // Each joint can only move a certain limit at once -> scale increments
  const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < joint_models.size(); ++i)
  {
    double scale = 1.0;
    double max_val = final_increments_.col(i).maxCoeff();
    double min_val = final_increments_.col(i).minCoeff();

    double max_scale = parameters_->joint_update_limit_ / fabs(max_val);
    double min_scale = parameters_->joint_update_limit_ / fabs(min_val);

    if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;

    group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
  }
}

void ChompOptimizer::updateFullTrajectory()
{
  // Write back from group_trajectory_ to the full_trajectory_
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void ChompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i = 0; i < num_joints_; ++i)
  {
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  }
  std::cout << "Cost = " << cost << std::endl;
}

double ChompOptimizer::getTrajectoryCost()
{
  return getSmoothnessCost() + getCollisionCost();
}

double ChompOptimizer::getSmoothnessCost()
{
  double smoothness_cost = 0.0;
  for (int i = 0; i < num_joints_; ++i)
  {
    smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  }
  return parameters_->smoothness_cost_weight_ * smoothness_cost;
}

double ChompOptimizer::getCollisionCost()
{
  double collision_cost = 0.0;
  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  // Sum over free variables
  for (int i = free_vars_start_; i <= free_vars_end_; ++i)
  {
    double state_collision_cost = 0.0;
    for (int j = 0; j < num_collision_points_; ++j)
    {
      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
    }
    collision_cost += state_collision_cost;
    if (state_collision_cost > worst_collision_cost)
    {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }
  return parameters_->obstacle_cost_weight_ * collision_cost;
}

void ChompOptimizer::computeJointProperties(int trajectory_point)
{
  // For each joint, compute global transforms for the axis and position
  for (int j = 0; j < num_joints_; ++j)
  {
    const moveit::core::JointModel* joint_model = state_.getJointModel(joint_names_[j]);
    const moveit::core::RevoluteJointModel* revolute_joint =
        dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
    const moveit::core::PrismaticJointModel* prismatic_joint =
        dynamic_cast<const moveit::core::PrismaticJointModel*>(joint_model);

    std::string parent_link_name = joint_model->getParentLinkModel()->getName();
    std::string child_link_name  = joint_model->getChildLinkModel()->getName();

    Eigen::Isometry3d joint_transform =
        state_.getGlobalLinkTransform(parent_link_name) *
        (robot_model_->getLinkModel(child_link_name)->getJointOriginTransform() *
         state_.getJointTransform(joint_model));

    Eigen::Vector3d axis;
    if (revolute_joint)
    {
      axis = revolute_joint->getAxis();
    }
    else if (prismatic_joint)
    {
      axis = prismatic_joint->getAxis();
    }
    else
    {
      axis = Eigen::Vector3d::Identity(); // e.g., multi-dof or fixed joint
    }

    // Convert axis into world frame
    axis = joint_transform * axis;

    joint_axes_[trajectory_point][j]      = axis;
    joint_positions_[trajectory_point][j] = joint_transform.translation();
  }
}

template <typename Derived>
void ChompOptimizer::getJacobian(int trajectory_point,
                                 Eigen::Vector3d& collision_point_pos,
                                 std::string& jointName,
                                 Eigen::MatrixBase<Derived>& jacobian) const
{
  // For each joint, check if it's a parent of jointName in the kinematic chain
  for (int j = 0; j < num_joints_; ++j)
  {
    if (isParent(jointName, joint_names_[j]))
    {
      // The column is the cross-product of the joint axis with (collision_point - joint_position)
      Eigen::Vector3d column = joint_axes_[trajectory_point][j].cross(
          collision_point_pos - joint_positions_[trajectory_point][j]);

      jacobian.col(j)[0] = column.x();
      jacobian.col(j)[1] = column.y();
      jacobian.col(j)[2] = column.z();
    }
    else
    {
      jacobian.col(j)[0] = 0.0;
      jacobian.col(j)[1] = 0.0;
      jacobian.col(j)[2] = 0.0;
    }
  }
}

void ChompOptimizer::handleJointLimits()
{
  // Check each joint for bounding
  const std::vector<const moveit::core::JointModel*> joint_models = joint_model_group_->getActiveJointModels();
  for (size_t joint_i = 0; joint_i < joint_models.size(); ++joint_i)
  {
    const moveit::core::JointModel* joint_model = joint_models[joint_i];

    // Revolute: if continuous, ignore limit
    if (joint_model->getType() == moveit::core::JointModel::REVOLUTE)
    {
      const moveit::core::RevoluteJointModel* revolute_joint =
          dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model);
      if (revolute_joint && revolute_joint->isContinuous())
      {
        continue;
      }
    }

    // Gather global bounds for this joint
    const moveit::core::JointModel::Bounds& bounds = joint_model->getVariableBounds();

    double joint_max = -DBL_MAX;
    double joint_min = DBL_MAX;

    for (const moveit::core::VariableBounds& bound : bounds)
    {
      if (bound.min_position_ < joint_min)
        joint_min = bound.min_position_;
      if (bound.max_position_ > joint_max)
        joint_max = bound.max_position_;
    }

    // Repeatedly fix any points that violate bounds
    bool violation = false;
    int count = 0;

    do
    {
      double max_abs_violation = 1e-6;
      double max_violation     = 0.0;
      int max_violation_index  = 0;
      violation = false;

      for (int i = free_vars_start_; i <= free_vars_end_; ++i)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        // Over upper bound
        if (group_trajectory_(i, joint_i) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint_i);
          absolute_amount = fabs(amount);
        }
        // Under lower bound
        else if (group_trajectory_(i, joint_i) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint_i);
          absolute_amount = fabs(amount);
        }

        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        // We penalize the free variable index with the violation
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier =
            max_violation / joint_costs_[joint_i].getQuadraticCostInverse()(free_var_index, free_var_index);

        group_trajectory_.getFreeJointTrajectoryBlock(joint_i) +=
            multiplier * joint_costs_[joint_i].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    }
    while (violation);
  }
}

void ChompOptimizer::performForwardKinematics()
{
  double inv_time     = 1.0 / group_trajectory_.getDiscretization();
  double inv_time_sq  = inv_time * inv_time;

  // If first iteration, compute from 0..end. Otherwise, just from free_vars_start_..free_vars_end_
  int start = (iteration_ == 0) ? 0 : free_vars_start_;
  int end   = (iteration_ == 0) ? (num_vars_all_ - 1) : free_vars_end_;

  is_collision_free_ = true;

  auto total_dur = std::chrono::duration<double>::zero();

  // For each trajectory index i
  for (int i = start; i <= end; ++i)
  {
    // Set state from the i-th row of group_trajectory_
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = planning_group_;

    setRobotStateFromPoint(group_trajectory_, i);

    auto grad_start = std::chrono::system_clock::now();
    hy_env_->getCollisionGradients(req, res, state_, nullptr, gsr_);
    total_dur += (std::chrono::system_clock::now() - grad_start);

    computeJointProperties(i);
    state_is_in_collision_[i] = false;

    // Fill in collision data for each sphere
    {
      size_t j = 0;
      for (const collision_detection::GradientInfo& info : gsr_->gradients_)
      {
        for (size_t k = 0; k < info.sphere_locations.size(); ++k)
        {
          collision_point_pos_eigen_[i][j][0] = info.sphere_locations[k].x();
          collision_point_pos_eigen_[i][j][1] = info.sphere_locations[k].y();
          collision_point_pos_eigen_[i][j][2] = info.sphere_locations[k].z();

          collision_point_potential_[i][j] =
              getPotential(info.distances[k], info.sphere_radii[k], parameters_->min_clearance_);

          collision_point_potential_gradient_[i][j][0] = info.gradients[k].x();
          collision_point_potential_gradient_[i][j][1] = info.gradients[k].y();
          collision_point_potential_gradient_[i][j][2] = info.gradients[k].z();

          point_is_in_collision_[i][j] = (info.distances[k] - info.sphere_radii[k] < info.sphere_radii[k]);
          if (point_is_in_collision_[i][j])
          {
            state_is_in_collision_[i] = true;
            is_collision_free_ = false;
          }
          j++;
        }
      }
    }
  }

  // RCLCPP_INFO(LOGGER, "Total gradient gathering took %f s for %d points", total_dur.count(), end - start + 1);

  // Compute velocities and accelerations using finite-differencing for each collision point
  for (int i = free_vars_start_; i <= free_vars_end_; ++i)
  {
    for (int j = 0; j < num_collision_points_; ++j)
    {
      collision_point_vel_eigen_[i][j].setZero();
      collision_point_acc_eigen_[i][j].setZero();

      for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; ++k)
      {
        collision_point_vel_eigen_[i][j] +=
            inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2] * collision_point_pos_eigen_[i + k][j];
        collision_point_acc_eigen_[i][j] +=
            inv_time_sq * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2] * collision_point_pos_eigen_[i + k][j];
      }

      // Norm of velocity
      collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
    }
  }
}

void ChompOptimizer::setRobotStateFromPoint(ChompTrajectory& group_trajectory, int i)
{
  const Eigen::MatrixXd::RowXpr& point = group_trajectory.getTrajectoryPoint(i);

  std::vector<double> joint_states;
  joint_states.reserve(group_trajectory.getNumJoints());

  for (size_t j = 0; j < group_trajectory.getNumJoints(); ++j)
  {
    joint_states.push_back(point(0, j));
  }

  state_.setJointGroupPositions(planning_group_, joint_states);
  state_.update();
}

void ChompOptimizer::perturbTrajectory()
{
  // This function is unused by default, but included for possible random escapes from local minima.
  if (worst_collision_cost_state_ < 0)
    return;

  int mid_point = worst_collision_cost_state_;
  moveit::core::RobotState random_state = state_;
  const moveit::core::JointModelGroup* planning_group_ptr = state_.getJointModelGroup(planning_group_);
  random_state.setToRandomPositions(planning_group_ptr);

  std::vector<double> vals;
  random_state.copyJointGroupPositions(planning_group_, vals);
  Eigen::Map<Eigen::VectorXd> random_matrix(vals.data(), vals.size());

  // Convert the state into an increment
  random_matrix -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();

  random_matrix = (Eigen::MatrixXd::Identity(num_joints_, num_joints_)
                   - joint_state_velocities_ * joint_state_velocities_.transpose())
                  * random_matrix;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i = 0; i < num_joints_; ++i)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
  }
}


}  // namespace chomp
