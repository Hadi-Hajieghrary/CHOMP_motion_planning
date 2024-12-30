#pragma once

#include "my_chomp_motion_planner/chomp_parameters.hpp"
#include "my_chomp_motion_planner/chomp_trajectory.hpp"
#include "my_chomp_motion_planner/chomp_utils.hpp"
#include <moveit/collision_distance_field/collision_env_hybrid.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <map>

#include <eigen3/Eigen/LU>
#include <algorithm>
#include <iterator>

namespace chomp
{

/**
 * \brief Represents the smoothness cost for CHOMP, for a single joint
 *
 * This class holds matrices needed for computing the smoothness cost of 
 * a joint trajectory (quad_cost_full_, quad_cost_, quad_cost_inv_).
 * The main usage is:
 *   1) Construct an instance with a given trajectory, derivative costs, and optional ridge factor.
 *   2) Retrieve cost or derivatives via getCost() and getDerivative().
 *   3) Scale costs if needed via scale().
 */
class ChompCost
{
public:
  /**
   * \brief Constructor for ChompCost
   * \param trajectory           A reference to the ChompTrajectory
   * \param derivative_costs     Vector of weighting coefficients for different orders of derivative
   * \param ridge_factor         Additional ridge (diagonal) term to ensure numerical stability
   */
  ChompCost(const ChompTrajectory& trajectoryr, const std::vector<double>& derivative_costs, double ridge_factor = 0.0);

  /**
   * \brief Destructor
   */
  virtual ~ChompCost();

  /**
   * \brief Compute the derivative of the cost wrt. the trajectory
   *
   * This function uses the \a quad_cost_full_ matrix to compute the
   * gradient of the smoothness cost. The final derivative is given by
   *    derivative = 2 * quad_cost_full_ * joint_trajectory
   *
   * \tparam Derived             Template type for the derivative output (Eigen-compatible)
   * \param joint_trajectory     The column vector representing the joint's configuration over time
   * \param derivative           Output parameter storing the result
   */
  template <typename Derived>
  void getDerivative(const Eigen::MatrixXd::ColXpr& joint_trajectory, Eigen::MatrixBase<Derived>& derivative) const;

  /**
   * \brief Get the inverse of the (free variables) quadratic cost matrix
   * \return const reference to quad_cost_inv_
   */
  const Eigen::MatrixXd& getQuadraticCostInverse() const;

  /**
   * \brief Get the (free variables) quadratic cost matrix
   * \return const reference to quad_cost_
   */
  const Eigen::MatrixXd& getQuadraticCost() const;

  /**
   * \brief Compute the smoothness cost for a given joint_trajectory
   *
   * This is simply the quadratic form:
   *      cost = joint_trajectory^T * quad_cost_full_ * joint_trajectory
   *
   * \param joint_trajectory The column vector representing the joint's configuration over time
   * \return The smoothness cost
   */
  double getCost(const Eigen::MatrixXd::ColXpr& joint_trajectory) const;

  /**
   * \brief Returns the maximum coefficient of quad_cost_inv_
   * \return The maximum coefficient value in quad_cost_inv_
   */
  double getMaxQuadCostInvValue() const;

  /**
   * \brief Scales the cost matrices by a given factor
   *
   * Specifically, if we scale by \a scale:
   *   quad_cost_inv_ *= 1.0/scale
   *   quad_cost_     *= scale
   *   quad_cost_full_ *= scale
   *
   * \param scale Scaling factor
   */
  void scale(double scale);

private:
  /**
   * \brief Full NxN quadratic cost matrix for all (boundary + free) variables
   */
  Eigen::MatrixXd quad_cost_full_;

  /**
   * \brief Quadratic cost matrix for only the free variables
   */
  Eigen::MatrixXd quad_cost_;

  /**
   * \brief Inverse of the cost matrix for only the free variables
   */
  Eigen::MatrixXd quad_cost_inv_;

  /**
   * \brief Constructs an (size x size) matrix that applies a finite-difference rule.
   * 
   * \param size        The dimension of the matrix (corresponds to number of trajectory points)
   * \param diff_rule   A pointer to a finite-difference rule (e.g., [-1, 2, -1], etc.)
   * \return            The resulting finite-difference matrix
   */
  Eigen::MatrixXd getDiffMatrix(int size, const double* diff_rule) const;
};

// ===================== Inline / Template Implementations =====================

template <typename Derived>
void ChompCost::getDerivative(const Eigen::MatrixXd::ColXpr& joint_trajectory,
                              Eigen::MatrixBase<Derived>& derivative) const
{
  // Multiply by 2.0 inside because we want 2 * quad_cost_full_ * joint_trajectory
  // Using Eigen expression templates to avoid unnecessary copies:
  derivative = quad_cost_full_ * (2.0 * joint_trajectory);
}

inline const Eigen::MatrixXd& ChompCost::getQuadraticCostInverse() const
{
  return quad_cost_inv_;
}

inline const Eigen::MatrixXd& ChompCost::getQuadraticCost() const
{
  return quad_cost_;
}

inline double ChompCost::getCost(const Eigen::MatrixXd::ColXpr& joint_trajectory) const
{
  // Quadratic form: x^T * A * x
  // .dot(...) is short for x^T * y for vectors x, y
  // (quad_cost_full_ * joint_trajectory) yields a vector, which is then dotted with joint_trajectory.
  return joint_trajectory.dot(quad_cost_full_ * joint_trajectory);
}



/**
 * \class ChompOptimizer
 * \brief Optimizes a given trajectory (ChompTrajectory) with respect to smoothness and collision avoidance.
 *
 * This class implements the CHOMP (Covariant Hamiltonian Optimization for Motion Planning) algorithm,
 * which iterates over a trajectory in joint space to reduce both smoothness and collision costs.
 */
class ChompOptimizer
{
public:
  /**
   * \brief Constructor
   * \param[in] trajectory        The full trajectory to be optimized
   * \param[in] planning_scene    Pointer to the planning scene (includes collision environment)
   * \param[in] planning_group    Name of the planning group to optimize
   * \param[in] parameters        Pointer to a ChompParameters instance controlling optimization settings
   * \param[in] start_state       RobotState representing the start configuration
   */
  ChompOptimizer(ChompTrajectory* trajectory,
                 const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const std::string& planning_group,
                 const ChompParameters* parameters,
                 const moveit::core::RobotState& start_state);

  /**
   * \brief Destructor
   */
  virtual ~ChompOptimizer();

  /**
   * \brief Main entrypoint to run the CHOMP optimization
   * \param Optional Storage to return the intermediate Trajectories and Associated Cost Values
   * \return true if an optimal collision-free path is found, false otherwise
   */
  bool optimize(std::vector<std::pair<ChompTrajectory, double>>& intermediate_results);


  /**
   * \brief Destroy or clean up resources (currently unused)
   */
  inline void destroy()
  {
    // Nothing for now.
  }

  /**
   * \brief Indicates if the optimizer was successfully initialized
   * \return true if optimizer was initialized, false otherwise
   */
  bool isInitialized() const
  {
    return initialized_;
  }

  /**
   * \brief Indicates if the last iteration ended with a collision-free trajectory
   * \return true if the final trajectory was collision-free, false otherwise
   */
  bool isCollisionFree() const
  {
    return is_collision_free_;
  }

private:
  /**
   * \brief Computes the potential function used for collision cost
   *
   * If the distance is outside the clearance, cost is 0.
   * Otherwise, transitions quadratically if 0 < d < clearance,
   * and linearly if d < 0.
   *
   * \param[in] field_distance  Distance from obstacle
   * \param[in] radius         Collision object radius
   * \param[in] clearance      Minimum allowed clearance
   * \return The collision potential
   */
  inline double getPotential(double field_distance, double radius, double clearance)
  {
    double d = field_distance - radius;

    if (d >= clearance)  // everything is fine
    {
      return 0.0;
    }
    else if (d >= 0.0)  // transition phase, no collision yet
    {
      // 0.5 * (d - clearance)^2 / clearance
      const double diff = (d - clearance);
      const double gradient_magnitude = diff / clearance;
      return 0.5 * gradient_magnitude * diff;
    }
    else  // d < 0.0 => collision
    {
      // linearly increase, starting from 0.5 * clearance
      return -d + 0.5 * clearance;
    }
  }

  /**
   * \brief Computes the Jacobian for a collision point relative to each joint
   *
   * Given the (x,y,z) position of the collision sphere, this function determines
   * how that point moves as each joint changes, effectively computing \f$ \frac{\partial x}{\partial \theta_j} \f$.
   *
   * \tparam Derived  Eigen matrix type for \a jacobian
   * \param[in]  trajectoryPoint     Index of the trajectory point
   * \param[in]  collision_point_pos The 3D position of the collision point
   * \param[in]  jointName          The child link/joint name where the collision sphere is attached
   * \param[out] jacobian           Output matrix (3 x num_joints_)
   */
  template <typename Derived>
  void getJacobian(int trajectoryPoint,
                   Eigen::Vector3d& collision_point_pos,
                   std::string& jointName,
                   Eigen::MatrixBase<Derived>& jacobian) const;

  /**
   * \brief Sets the robot's state_ to match a given row (time index) in \a group_trajectory
   * \param[in] group_trajectory  Reference to the partial trajectory (group scope)
   * \param[in] i                 The time index in the trajectory
   */
  void setRobotStateFromPoint(ChompTrajectory& group_trajectory, int i);

  // ------------------- Private Data Members -------------------

  int num_joints_;              /**< Number of joints being optimized */
  int num_vars_free_;           /**< Number of free variables in time dimension (not counting boundary points) */
  int num_vars_all_;            /**< Total number of points in the trajectory (including boundaries) */
  int num_collision_points_;    /**< Total number of collision points across the robot */
  int free_vars_start_;         /**< First index in the free region (time dimension) */
  int free_vars_end_;           /**< Last index in the free region (time dimension) */
  int iteration_;               /**< Counter for the current iteration in optimize() */
  unsigned int collision_free_iteration_; /**< Counter to track how many steps we've been collision-free in a row */

  ChompTrajectory* full_trajectory_;  /**< The full trajectory to be optimized (shared pointer) */
  const moveit::core::RobotModelConstPtr& robot_model_; /**< The robot model used for kinematics and bounds */
  std::string planning_group_;                           /**< Planning group name */
  const ChompParameters* parameters_;                    /**< Optimization parameters (e.g. cost weights, etc.) */
  ChompTrajectory group_trajectory_;                     /**< The "group" trajectory being optimized */
  planning_scene::PlanningSceneConstPtr planning_scene_; /**< Planning scene pointer */
  moveit::core::RobotState state_;                       /**< Robot state used to perform forward kinematics */
  moveit::core::RobotState start_state_;                 /**< Starting robot state */
  const moveit::core::JointModelGroup* joint_model_group_; /**< Pointer to the joint model group being optimized */
  const collision_detection::CollisionEnvHybrid* hy_env_;   /**< Hybrid collision environment for gradient queries */

  std::vector<ChompCost> joint_costs_;                   /**< Smoothness cost objects, one per joint */
  collision_detection::GroupStateRepresentationPtr gsr_;  /**< Collision gradients from the collision environment */
  bool initialized_;                                      /**< Flag to indicate if initialize() succeeded */

  std::vector<std::vector<std::string>> collision_point_joint_names_; /**< For each traj point, names of the joints
                                                                            associated with collision spheres */
  std::vector<EigenSTL::vector_Vector3d> collision_point_pos_eigen_;  /**< (X,Y,Z) positions of collision spheres */
  std::vector<EigenSTL::vector_Vector3d> collision_point_vel_eigen_;  /**< Estimated velocities of collision spheres */
  std::vector<EigenSTL::vector_Vector3d> collision_point_acc_eigen_;  /**< Estimated accelerations of collision spheres */
  std::vector<std::vector<double>> collision_point_potential_;        /**< Collision cost for each sphere */
  std::vector<std::vector<double>> collision_point_vel_mag_;          /**< Magnitude of sphere velocities */
  std::vector<EigenSTL::vector_Vector3d> collision_point_potential_gradient_; /**< Gradient wrt. sphere positions */
  std::vector<EigenSTL::vector_Vector3d> joint_axes_;                 /**< Axes of rotation/translation for each joint */
  std::vector<EigenSTL::vector_Vector3d> joint_positions_;            /**< Positions of each joint */
  Eigen::MatrixXd group_trajectory_backup_;   /**< Backup of the group trajectory, used for fallback if needed */
  Eigen::MatrixXd best_group_trajectory_;     /**< Best trajectory found so far */
  double best_group_trajectory_cost_;         /**< Cost associated with best_group_trajectory_ */
  int last_improvement_iteration_;            /**< Iteration number at which we last updated best_group_trajectory_ */
  unsigned int num_collision_free_iterations_; /**< Used to exit early if collision-free for multiple steps */

  // HMC / Stochastic components:
  Eigen::MatrixXd momentum_;       /**< HMC: current momentum (unused if Hamiltonian MC is off) */
  Eigen::MatrixXd random_momentum_;/**< HMC: random initial momentum (unused if Hamiltonian MC is off) */
  Eigen::VectorXd random_joint_momentum_; /**< HMC: temporary 1D momentum vector for a single joint */
  std::vector<MultivariateGaussian> multivariate_gaussian_; /**< HMC: gaussians for joint sampling */
  double stochasticity_factor_;    /**< HMC: scales the random momentum each iteration */

  std::vector<int> state_is_in_collision_;                /**< Collision indicator per trajectory point */
  std::vector<std::vector<int>> point_is_in_collision_;   /**< Detailed collision indicators for each point and sphere */
  bool is_collision_free_;                                 /**< If the final trajectory is collision-free */
  double worst_collision_cost_state_;                      /**< (debug) index of the worst cost in the trajectory */

  // Temporary accumulators for gradient calculations:
  Eigen::MatrixXd smoothness_increments_;  /**< Smoothness gradient contribution */
  Eigen::MatrixXd collision_increments_;   /**< Collision gradient contribution */
  Eigen::MatrixXd final_increments_;       /**< Combined total gradient contribution */

  // Additional temporary matrices/vectors:
  Eigen::VectorXd smoothness_derivative_;       /**< Temporary vector for storing derivative from ChompCost */
  Eigen::MatrixXd jacobian_;                    /**< Temporary 3xN matrix for partial derivatives of collision point */
  Eigen::MatrixXd jacobian_pseudo_inverse_;     /**< Pseudoinverse of jacobian_ for advanced gradient calculation */
  Eigen::MatrixXd jacobian_jacobian_tranpose_;  /**< Intermediate matrix = J * J^T */
  Eigen::VectorXd random_state_;                /**< For randomizing / perturbing states */
  Eigen::VectorXd joint_state_velocities_;      /**< For storing velocities at a given point in the trajectory */

  std::vector<std::string> joint_names_; /**< List of active joint names in the planning group */
  std::map<std::string, std::map<std::string, bool>> joint_parent_map_; /**< Mapping from a joint to its parent joints */

  /**
   * \brief Determines if a link/joint is a parent (ancestor) of another in the kinematic chain
   * \param[in] childLink   Name of the child link/joint
   * \param[in] parentLink  Name of the potential ancestor
   * \return true if \a parentLink is an ancestor of \a childLink, false otherwise
   */
  inline bool isParent(const std::string& childLink, const std::string& parentLink) const
  {
    // Base case
    if (childLink == parentLink)
    {
      return true;
    }

    // If childLink is not in map, it can't have a parent
    if (joint_parent_map_.find(childLink) == joint_parent_map_.end())
    {
      // Could log an error or warning here
      return false;
    }

    // If parentLink is stored in childLink's parent map and is true => is a parent
    const std::map<std::string, bool>& parents = joint_parent_map_.at(childLink);
    auto it = parents.find(parentLink);
    return (it != parents.end() && it->second);
  }

  /**
   * \brief Builds the mapping of parents for a given \a model
   *
   * This recursively ascends the kinematic chain from \a model to the root,
   * marking each joint in the path as a parent.
   */
  void registerParents(const moveit::core::JointModel* model);

  /**
   * \brief One-time initialization of data structures and collision environment
   */
  void initialize();

  /**
   * \brief Accumulates smoothness derivatives from each joint (ChompCost) into smoothness_increments_
   */
  void calculateSmoothnessIncrements();

  /**
   * \brief Accumulates collision derivatives from the collision environment into collision_increments_
   */
  void calculateCollisionIncrements();

  /**
   * \brief Combines smoothness_increments_ and collision_increments_ into final_increments_
   *        using the joint quadratic cost inverse.
   */
  void calculateTotalIncrements();

  /**
   * \brief Performs the forward kinematics on each trajectory point, updates collision data
   *
   * This sets the \a state_ to each row of \a group_trajectory_ and queries the hybrid collision environment
   * to get gradient information. Also estimates velocity and acceleration via finite differencing.
   */
  void performForwardKinematics();

  /**
   * \brief Adds final_increments_ to the group_trajectory_, scaled by a joint update limit
   */
  void addIncrementsToTrajectory();

  /**
   * \brief Pushes the local group_trajectory_ back into the full_trajectory_ (for multi-group CHOMP)
   */
  void updateFullTrajectory();

  /**
   * \brief Outputs debug info about the smoothness cost (unused)
   */
  void debugCost();

  /**
   * \brief Enforces joint limits by adjusting points that fall outside bounds
   */
  void handleJointLimits();

  /**
   * \brief Computes the total cost (smoothness + collision)
   */
  double getTrajectoryCost();

  /**
   * \brief Computes just the smoothness part of the cost
   */
  double getSmoothnessCost();

  /**
   * \brief Computes just the collision part of the cost
   */
  double getCollisionCost();

  /**
   * \brief Randomly perturbs the trajectory near the worst collision state (unused by default)
   */
  void perturbTrajectory();

  /**
   * \brief Checks if the current best_group_trajectory_ is collision-free w.r.t. the planning_scene
   *        with mesh-to-mesh checking
   */
  bool isCurrentTrajectoryMeshToMeshCollisionFree() const;

  /**
   * \brief Calculates the pseudo-inverse of \a jacobian_ and stores in jacobian_pseudo_inverse_
   *
   * Used to compute advanced collision gradient updates if parameters_->use_pseudo_inverse_ is true.
   */
  void calculatePseudoInverse();

  // ----------- Possibly used for HMC, commented out in the .cpp -----------
  /*
  void getRandomMomentum();
  void updateMomentum();
  void updatePositionFromMomentum();
  */

  /**
   * \brief Computes needed transforms for each joint at a specific trajectory point
   * \param[in] trajectory_point The row (time index) in the trajectory
   */
  void computeJointProperties(int trajectory_point);

  /**
   * Advance the Optimization One Step
   * \return pair of  {collision cost, smoothness cost}
   */
  std::pair<double, double> stepOptimization();

  // ------------------- End Private Data/Methods -------------------
};

}  // namespace chomp
