#pragma once

#include <my_chomp_motion_planner/chomp_utils.hpp>
#include <moveit/robot_model/robot_model.h>

#include <eigen3/Eigen/Core>
#include <moveit_msgs/msg/motion_plan_detailed_response.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace chomp
{
/**
 * \class ChompTrajectory
 * \brief Represents a discretized joint-space trajectory for use in CHOMP.
 *
 * This class stores a matrix of joint positions over time:
 *    - Rows correspond to trajectory points (time steps)
 *    - Columns correspond to different joints
 * 
 * The \a start_index_ and \a end_index_ delimit the portion of the trajectory
 * that can be modified by the optimizer (i.e., free variables).
 */
class ChompTrajectory
{
public:
  /**
   * \brief Constructs a trajectory for a given robot model, trajectory \a duration, and \a discretization.
   * \param[in] robot_model   Shared pointer to the robot model
   * \param[in] duration      The total duration (in seconds) of the trajectory
   * \param[in] discretization The time step (seconds) between trajectory points
   * \param[in] group_name    The planning group name
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                  double duration,
                  double discretization,
                  const std::string& group_name);

  /**
   * \brief Constructs a trajectory for a given robot model, \a num_points, and \a discretization.
   * \param[in] robot_model   Shared pointer to the robot model
   * \param[in] num_points    Number of discrete points in the trajectory
   * \param[in] discretization The time step (seconds) between trajectory points
   * \param[in] group_name    The planning group name
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                  size_t num_points,
                  double discretization,
                  const std::string& group_name);

  /**
   * \brief Create a new trajectory containing only the subset of interest, adding padding for derivative rules.
   *
   * This is typically used to create a per-group trajectory, or to ensure enough boundary points
   * for finite-differencing (e.g., a 7-point stencil needs 3 boundary points on each side).
   * 
   * \param[in] source_traj      Source (full) trajectory to copy from
   * \param[in] group_name       The planning group name
   * \param[in] diff_rule_length The length of the derivative rule (e.g., 7 for jerk)
   */
  ChompTrajectory(const ChompTrajectory& source_traj,
                  const std::string& group_name,
                  int diff_rule_length);

  /**
   * \brief Constructs a trajectory from a \a JointTrajectory message for the specified group.
   * \param[in] robot_model   Shared pointer to the robot model
   * \param[in] group_name    The planning group name
   * \param[in] traj          The ROS trajectory message
   */
  ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                  const std::string& group_name,
                  const trajectory_msgs::msg::JointTrajectory& traj);

  /**
   * \brief Virtual destructor
   */
  virtual ~ChompTrajectory() = default;

  /**
   * \brief Access the (row, column) element of the trajectory by reference
   *
   * This returns a modifiable reference to the requested element:
   * \param[in] traj_point Index of the trajectory point (row)
   * \param[in] joint      Joint index (column)
   * \return A reference to the element in the matrix
   */
  double& operator()(size_t traj_point, size_t joint);

  /**
   * \brief Access the (row, column) element of the trajectory (read-only)
   *
   * \param[in] traj_point Index of the trajectory point (row)
   * \param[in] joint      Joint index (column)
   * \return The corresponding element (constant) in the trajectory
   */
  double operator()(size_t traj_point, size_t joint) const;

  /**
   * \brief Returns a row expression for the specified trajectory point
   * \param[in] traj_point The row index
   * \return A row expression (Eigen::MatrixXd::RowXpr)
   */
  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  /**
   * \brief Returns a column expression for the specified joint
   * \param[in] joint The column index
   * \return A column expression (Eigen::MatrixXd::ColXpr)
   */
  Eigen::MatrixXd::ColXpr getJointTrajectory(int joint);

  /**
   * \brief The total number of points (rows) in the trajectory
   * \return num_points_
   */
  size_t getNumPoints() const;

  /**
   * \brief The number of free points that can be optimized (from start_index_ to end_index_)
   * \return (end_index_ - start_index_) + 1
   */
  size_t getNumFreePoints() const;

  /**
   * \brief The number of joints (columns) in each trajectory point
   * \return num_joints_
   */
  size_t getNumJoints() const;

  /**
   * \brief Discretization time step used in this trajectory
   * \return discretization_
   */
  double getDiscretization() const;

  /**
   * \brief Generates a minimum-jerk interpolation from start_index_ to end_index_
   *
   * Points before start_index_ and after end_index_ are left unchanged.
   */
  void fillInMinJerk();

  /**
   * \brief Generates a linear interpolation from start_index_ to end_index_
   *
   * Points before start_index_ and after end_index_ are left unchanged.
   */
  void fillInLinearInterpolation();

  /**
   * \brief Generates a cubic interpolation from start_index_ to end_index_
   *
   * Points before start_index_ and after end_index_ are left unchanged.
   */
  void fillInCubicInterpolation();

  /**
   * \brief Fills this trajectory from a \a RobotTrajectory object
   *
   * This typically comes from OMPL or another planner. 
   * It interpolates over the entire requested range in \a trajectory.
   *
   * \param[in] trajectory  The input RobotTrajectory
   * \return true if the input trajectory had >=2 waypoints; false otherwise
   */
  bool fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory);

  /**
   * \brief Assigns the RobotState \a source to row \a chomp_trajectory_point in the internal matrix
   *
   * \param[in] source                    The source RobotState
   * \param[in] chomp_trajectory_point    Index (row) to which data is assigned
   * \param[in] group                     The JointModelGroup specifying which joints to copy
   */
  void assignCHOMPTrajectoryPointFromRobotState(const moveit::core::RobotState& source,
                                                size_t chomp_trajectory_point,
                                                const moveit::core::JointModelGroup* group);

  /**
   * \brief Sets the start and end index that define the "free" portion of the trajectory
   *
   * Everything before start_index is considered fixed (non-optimizable),
   * and everything after end_index is also fixed.
   */
  void setStartEndIndex(size_t start_index, size_t end_index);

  /**
   * \brief Get the start index
   * \return start_index_
   */
  size_t getStartIndex() const;

  /**
   * \brief Get the end index
   * \return end_index_
   */
  size_t getEndIndex() const;

  /**
   * \brief Provides direct (mutable) access to the entire underlying matrix
   * \return Reference to trajectory_ matrix
   */
  Eigen::MatrixXd& getTrajectory();

  /**
   * \brief Returns an Eigen block corresponding to the free portion (start_index_ to end_index_)
   * \return A block of dimension [getNumFreePoints() x getNumJoints()]
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

  /**
   * \brief Returns an Eigen block for the free portion of a single joint
   * \param[in] joint Index of the joint (column)
   * \return A block of dimension [getNumFreePoints() x 1]
   */
  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeJointTrajectoryBlock(size_t joint);

  /**
   * \brief Updates this (full) trajectory from a (group) trajectory block
   *
   * Overwrites the rows from start_index_ to end_index_ with the corresponding block
   * in \a group_trajectory.
   */
  void updateFromGroupTrajectory(const ChompTrajectory& group_trajectory);

  /**
   * \brief Returns the index in the source (full) trajectory
   * \param[in] i The local index in this trajectory
   * \return Index in the original full trajectory
   */
  size_t getFullTrajectoryIndex(size_t i) const;

  /**
   * \brief Gets the joint velocities at \a traj_point using a finite-difference rule.
   *
   * Applies the velocity rule from \a DIFF_RULES[0].
   * \tparam Derived        Type of \a velocities (e.g., Eigen::VectorXd)
   * \param[in] traj_point  Index (row) of the trajectory
   * \param[out] velocities Output vector to fill with joint velocities
   */
  template <typename Derived>
  void getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities);

  /**
   * \brief The total duration of the trajectory in seconds
   * \return duration_
   */
  double getDuration() const;

private:
  /**
   * \brief Allocates/initializes trajectory_ matrix with the known dimensions
   */
  void init();

  std::string planning_group_name_;       /**< The planning group that this trajectory corresponds to */
  size_t num_points_;                     /**< Total number of points (rows) in the trajectory */
  size_t num_joints_;                     /**< Number of joints (columns) in each trajectory point */
  double discretization_;                 /**< Time step (seconds) between consecutive points */
  double duration_;                       /**< Total duration of the trajectory in seconds */
  Eigen::MatrixXd trajectory_;            /**< The actual matrix storing joint positions */
  size_t start_index_;                    /**< First index of the free region (inclusive) */
  size_t end_index_;                      /**< Last index of the free region (inclusive) */
  std::vector<size_t> full_trajectory_index_; /**< Mapping from local indices to the original (full) trajectory */

};  // class ChompTrajectory

// ====================== Inline Function Definitions ======================

inline double& ChompTrajectory::operator()(size_t traj_point, size_t joint)
{
  return trajectory_(traj_point, joint);
}

inline double ChompTrajectory::operator()(size_t traj_point, size_t joint) const
{
  return trajectory_(traj_point, joint);
}

inline Eigen::MatrixXd::RowXpr ChompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

inline Eigen::MatrixXd::ColXpr ChompTrajectory::getJointTrajectory(int joint)
{
  return trajectory_.col(joint);
}

inline size_t ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline size_t ChompTrajectory::getNumFreePoints() const
{
  // If start_index_ = 1 and end_index_ = num_points_ - 2, for example,
  // we have (end_index_ - start_index_) + 1 free points.
  return (end_index_ - start_index_) + 1;
}

inline size_t ChompTrajectory::getNumJoints() const
{
  return num_joints_;
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

inline void ChompTrajectory::setStartEndIndex(size_t start_index, size_t end_index)
{
  start_index_ = start_index;
  end_index_ = end_index;
}

inline size_t ChompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline size_t ChompTrajectory::getEndIndex() const
{
  return end_index_;
}

inline Eigen::MatrixXd& ChompTrajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ChompTrajectory::getFreeTrajectoryBlock()
{
  // Rows from start_index_ to end_index_, all columns
  return trajectory_.block(start_index_, 0, getNumFreePoints(), getNumJoints());
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreeJointTrajectoryBlock(size_t joint)
{
  // Rows from start_index_ to end_index_, a single column
  return trajectory_.block(start_index_, joint, getNumFreePoints(), 1);
}

inline size_t ChompTrajectory::getFullTrajectoryIndex(size_t i) const
{
  return full_trajectory_index_[i];
}

template <typename Derived>
void ChompTrajectory::getJointVelocities(size_t traj_point, Eigen::MatrixBase<Derived>& velocities)
{
  // Zero out the velocities vector
  velocities.setZero();

  // 1 / (time step)
  double inv_time = 1.0 / discretization_;

  // Apply velocity rule from DIFF_RULES[0], which is typically the finite difference
  // stencil for the first derivative.
  for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; ++k)
  {
    // Each row(...) is the row at (traj_point + k).
    // We multiply by inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2].
    velocities += (inv_time * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) *
                  trajectory_.row(traj_point + k).transpose();
  }
}

inline double ChompTrajectory::getDuration() const
{
  return duration_;
}

}  // namespace chomp