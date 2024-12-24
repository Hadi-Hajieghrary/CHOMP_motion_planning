#include <my_chomp_motion_planner/chomp_trajectory.hpp>

namespace chomp
{

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                 double duration,
                                 double discretization,
                                 const std::string& group_name)
  : ChompTrajectory(robot_model,
                    static_cast<size_t>(duration / discretization) + 1,
                    discretization,
                    group_name)
{
  // This constructor simply delegates to the second constructor.
}

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                 size_t num_points,
                                 double discretization,
                                 const std::string& group_name)
  : planning_group_name_(group_name)
  , num_points_(num_points)
  , discretization_(discretization)
  , duration_((num_points - 1) * discretization)
  , start_index_(1)
  , end_index_(num_points_ - 2)
{
  // Retrieve the JointModelGroup
  const moveit::core::JointModelGroup* model_group = robot_model->getJointModelGroup(planning_group_name_);
  // Number of active joints in this group
  num_joints_ = model_group->getActiveJointModels().size();

  // Allocate memory for the trajectory matrix
  init();
}

ChompTrajectory::ChompTrajectory(const ChompTrajectory& source_traj,
                                 const std::string& group_name,
                                 int diff_rule_length)
  : planning_group_name_(group_name)
  , discretization_(source_traj.discretization_)
{
  // Copy the number of joints from the source
  num_joints_ = source_traj.getNumJoints();

  // Compute how many extra points are needed on each side
  // to satisfy the derivative rule length (e.g., for a 7-point stencil).
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  // Compute the new number of points
  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  start_index_ = diff_rule_length - 1;
  end_index_   = (num_points_ - 1) - (diff_rule_length - 1);

  // The trajectory duration scales accordingly
  duration_ = (num_points_ - 1) * discretization_;

  // Allocate the memory for the new trajectory
  init();

  // Resize the mapping array (local -> full index)
  full_trajectory_index_.resize(num_points_);

  // Copy data from source_traj into this new expanded trajectory
  for (size_t i = 0; i < num_points_; ++i)
  {
    // The corresponding source index, clipped to [0, source_traj.num_points_ - 1]
    int source_traj_point = static_cast<int>(i) - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (static_cast<size_t>(source_traj_point) >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;

    // Store the mapping
    full_trajectory_index_[i] = source_traj_point;
    // Copy the row from the source trajectory
    getTrajectoryPoint(i) = const_cast<ChompTrajectory&>(source_traj)
                                .getTrajectoryPoint(source_traj_point);
  }
}

ChompTrajectory::ChompTrajectory(const moveit::core::RobotModelConstPtr& /*robot_model*/,
                                 const std::string& /*group_name*/,
                                 const trajectory_msgs::msg::JointTrajectory& /*traj*/)
{
  // Typically, one would parse the JointTrajectory message here and fill in
  // the internal trajectory_ matrix. This constructor is declared but not defined
  // in the provided snippet. You may implement it as needed.
  // For now, this is left as an example of potential usage.
}

void ChompTrajectory::init()
{
  // Initialize the trajectory matrix with the known dimensions
  trajectory_.resize(num_points_, num_joints_);
}

void ChompTrajectory::updateFromGroupTrajectory(const ChompTrajectory& group_trajectory)
{
  // Number of free variables in the time dimension
  size_t num_vars_free = end_index_ - start_index_ + 1;

  // Copy the block of data from group_trajectory
  trajectory_.block(start_index_, 0, num_vars_free, num_joints_) =
      group_trajectory.trajectory_.block(group_trajectory.start_index_, 0, num_vars_free, num_joints_);
}

void ChompTrajectory::fillInLinearInterpolation()
{
  // We linearly interpolate from (start_index_-1) to (end_index_+1) for each joint
  double start_index = start_index_ - 1;
  double end_index   = end_index_ + 1;

  // For each joint
  for (size_t i = 0; i < num_joints_; ++i)
  {
    // Slope = (value_at_end - value_at_start) / (end_index - 1)
    double theta = ((*this)(end_index, i) - (*this)(start_index, i)) / (end_index - 1);

    // Fill intermediate points
    for (size_t j = start_index + 1; j < end_index; ++j)
    {
      (*this)(j, i) = (*this)(start_index, i) + j * theta;
    }
  }
}

void ChompTrajectory::fillInCubicInterpolation()
{
  // Cubic interpolation from (start_index_-1) to (end_index_+1)
  double start_index = start_index_ - 1;
  double end_index   = end_index_ + 1;

  // A small time step for each sub-interval
  double dt = 0.001;

  // We'll compute up to t = (end_index - 1) * dt
  double total_time = (end_index - 1) * dt;
  std::vector<double> coeffs(4, 0.0);

  // For each joint
  for (size_t i = 0; i < num_joints_; ++i)
  {
    // Set up the typical cubic polynomial form:
    //   pos(t) = coeffs[0] + coeffs[1]*t + coeffs[2]*t^2 + coeffs[3]*t^3
    // with boundary conditions matching the two endpoints (start_index, end_index) and zero derivatives, etc.
    coeffs[0] = (*this)(start_index, i);
    coeffs[2] = (3.0 / (std::pow(total_time, 2))) * ((*this)(end_index, i) - (*this)(start_index, i));
    coeffs[3] = (-2.0 / (std::pow(total_time, 3))) * ((*this)(end_index, i) - (*this)(start_index, i));

    // Compute trajectory for each intermediate point j
    for (size_t j = start_index + 1; j < end_index; ++j)
    {
      double t = j * dt;
      (*this)(j, i) = coeffs[0]
                      + coeffs[2] * std::pow(t, 2)
                      + coeffs[3] * std::pow(t, 3);
    }
  }
}

void ChompTrajectory::fillInMinJerk()
{
  // Minimum jerk from (start_index_-1) to (end_index_+1)
  double start_index = start_index_ - 1;
  double end_index   = end_index_ + 1;

  // td array will store powers of total time = (end_index - start_index) * discretization_
  double td[6];
  td[0] = 1.0;
  td[1] = (end_index - start_index) * discretization_;

  for (unsigned int i = 2; i <= 5; ++i)
    td[i] = td[i - 1] * td[1]; // td[i] = (total_time)^i

  // We assume start and end velocity and acceleration are zero.
  // For each joint, compute the polynomial coefficients
  std::vector<std::array<double, 6>> coeff(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i)
  {
    double x0 = (*this)(start_index, i);
    double x1 = (*this)(end_index, i);

    coeff[i][0] = x0;  // constant term
    coeff[i][1] = 0.0; // no linear term
    coeff[i][2] = 0.0; // no quadratic term
    coeff[i][3] = (-20.0 * x0 + 20.0 * x1) / (2.0 * td[3]);
    coeff[i][4] = (30.0 * x0 - 30.0 * x1) / (2.0 * td[4]);
    coeff[i][5] = (-12.0 * x0 + 12.0 * x1) / (2.0 * td[5]);
  }

  // Fill in the intermediate points
  for (size_t i = start_index + 1; i < end_index; ++i)
  {
    double ti[6];
    ti[0] = 1.0;
    ti[1] = (i - start_index) * discretization_;
    for (unsigned int k = 2; k <= 5; ++k)
      ti[k] = ti[k - 1] * ti[1]; // ti[k] = (t)^k

    for (size_t j = 0; j < num_joints_; ++j)
    {
      (*this)(i, j) = 0.0;
      for (unsigned int k = 0; k <= 5; ++k)
      {
        (*this)(i, j) += ti[k] * coeff[j][k];
      }
    }
  }
}

bool ChompTrajectory::fillInFromTrajectory(const robot_trajectory::RobotTrajectory& trajectory)
{
  // Must have at least two waypoints (start and goal)
  if (trajectory.getWayPointCount() < 2)
    return false;

  // Convert from external (possibly arbitrary) number of states to exactly getNumPoints() states
  const size_t max_output_index = getNumPoints() - 1;         // last index of this trajectory
  const size_t max_input_index  = trajectory.getWayPointCount() - 1; // last index of incoming trajectory

  const moveit::core::JointModelGroup* group = trajectory.getGroup();

  // We'll interpolate between the waypoints in the RobotTrajectory
  moveit::core::RobotState interpolated(trajectory.getRobotModel());

  for (size_t i = 0; i <= max_output_index; ++i)
  {
    // fraction is i / max_output_index, scaled to [0, max_input_index]
    double fraction = static_cast<double>(i * max_input_index) / max_output_index;

    // Separate integer and fractional parts
    size_t prev_idx = static_cast<size_t>(std::trunc(fraction));
    fraction = fraction - prev_idx; // keep only fractional part

    // Next index is either prev_idx or prev_idx+1, clipped to max_input_index
    size_t next_idx = (prev_idx == max_input_index) ? prev_idx : prev_idx + 1;

    // Interpolate states at fraction in [0,1] between waypoints
    trajectory.getWayPoint(prev_idx).interpolate(trajectory.getWayPoint(next_idx),
                                                 fraction,
                                                 interpolated,
                                                 group);

    // Assign the interpolated values to row i of this trajectory
    assignCHOMPTrajectoryPointFromRobotState(interpolated, i, group);
  }

  return true;
}

void ChompTrajectory::assignCHOMPTrajectoryPointFromRobotState(const moveit::core::RobotState& source,
                                                               size_t chomp_trajectory_point_index,
                                                               const moveit::core::JointModelGroup* group)
{
  // Retrieve the row expression for the target row
  Eigen::MatrixXd::RowXpr target = getTrajectoryPoint(chomp_trajectory_point_index);

  // Make sure the dimensionalities match
  assert(group->getActiveJointModels().size() == static_cast<size_t>(target.cols()));

  // Write each active joint's value into the row
  size_t joint_index = 0;
  for (const moveit::core::JointModel* jm : group->getActiveJointModels())
  {
    // Typically each joint model has 1 variable if it’s a revolute or prismatic joint,
    // but multi-dof joints might have more. This code asserts it's 1:
    assert(jm->getVariableCount() == 1);

    // Copy the position from the RobotState
    target[joint_index++] = source.getVariablePosition(jm->getFirstVariableIndex());
  }
}

}  // namespace chomp
