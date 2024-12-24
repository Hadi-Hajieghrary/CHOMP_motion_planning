#include <my_chomp_motion_planner/chomp_optimizer.hpp>
#include <my_chomp_motion_planner/chomp_planner.hpp>
#include <my_chomp_motion_planner/chomp_trajectory.hpp>

#include <moveit/robot_state/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>  // for timing utilities
#include <cassert> // for assert()

namespace chomp
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("chomp_planner");

bool ChompPlanner::solve(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         const planning_interface::MotionPlanRequest& req,
                         const ChompParameters& params,
                         planning_interface::MotionPlanDetailedResponse& res) const
{
    const auto start_time = std::chrono::system_clock::now();

    // ------------------------- Basic Validation -------------------------
    if (!planning_scene)
    {
        RCLCPP_ERROR(LOGGER, "No planning scene provided. Cannot plan.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return false;
    }

    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);
    if (!start_state.satisfiesBounds())
    {
        RCLCPP_ERROR(LOGGER, "Start state violates joint limits.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    // ------------------------- Trajectory Initialization -------------------------
    constexpr double trajectory_duration = 3.0;
    constexpr double trajectory_discretization = 0.03;
    ChompTrajectory trajectory(planning_scene->getRobotModel(), trajectory_duration, trajectory_discretization, req.group_name);

    robotStateToArray(start_state, req.group_name, trajectory.getTrajectoryPoint(0));

    if (req.goal_constraints.size() != 1)
    {
        RCLCPP_ERROR(LOGGER, "Expected exactly 1 goal constraint, got: %zu", req.goal_constraints.size());
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    const auto& goal_constraints = req.goal_constraints[0];
    if (goal_constraints.joint_constraints.empty() ||
        !goal_constraints.position_constraints.empty() ||
        !goal_constraints.orientation_constraints.empty())
    {
        RCLCPP_ERROR(LOGGER, "Only pure joint-space goals are supported by CHOMP.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    const size_t goal_index = trajectory.getNumPoints() - 1;
    moveit::core::RobotState goal_state(start_state);

    // Use std::for_each to configure the goal state
    std::for_each(goal_constraints.joint_constraints.begin(),
                  goal_constraints.joint_constraints.end(),
                  [&goal_state](const auto& joint_constraint) {
                      goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
                  });

    if (!goal_state.satisfiesBounds())
    {
        RCLCPP_ERROR(LOGGER, "Goal state violates joint limits.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    robotStateToArray(goal_state, req.group_name, trajectory.getTrajectoryPoint(goal_index));

    // Adjust goal positions for continuous joints
    const auto* model_group = planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
    if (!model_group)
    {
        RCLCPP_ERROR(LOGGER, "JointModelGroup '%s' not found in robot model.", req.group_name.c_str());
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return false;
    }

    const auto& joint_models = model_group->getActiveJointModels();
    std::transform(joint_models.begin(), joint_models.end(),
                   trajectory.getTrajectoryPoint(goal_index).data(),
                   trajectory.getTrajectoryPoint(goal_index).data(),
                   [&trajectory, goal_index](const auto* joint_model, double& goal_value) {
                       if (const auto* revolute_joint = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model); revolute_joint && revolute_joint->isContinuous())
                       {
                           double start_val = trajectory(0, joint_model->getJointIndex());
                           return start_val + shortestAngularDistance(start_val, goal_value);
                       }
                       return goal_value;
                   });

    // ---------------------- Interpolation Method Selection ----------------------
    const auto& init_method = params.trajectory_initialization_method_;
    if (init_method == "quintic-spline")
    {
        trajectory.fillInMinJerk();
    }
    else if (init_method == "linear")
    {
        trajectory.fillInLinearInterpolation();
    }
    else if (init_method == "cubic")
    {
        trajectory.fillInCubicInterpolation();
    }
    else if (init_method == "fillTrajectory")
    {
        if (res.trajectory_.empty() || !res.trajectory_[0] || !trajectory.fillInFromTrajectory(*res.trajectory_[0]))
        {
            RCLCPP_ERROR(LOGGER, "Invalid input trajectory for fillTrajectory initialization method.");
            res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Invalid interpolation method: %s", init_method.c_str());
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return false;
    }

    RCLCPP_INFO(LOGGER, "CHOMP trajectory initialized via method: %s", init_method.c_str());

    // ---------------------- Optimization and Recovery Behavior ----------------------
    ChompParameters local_params = params;
    bool optimization_result = false;

    for (int attempt = 0; attempt <= params.max_recovery_attempts_; ++attempt)
    {
        if (attempt > 0)
        {
            local_params.setRecoveryParams(
                local_params.learning_rate_ + 0.02,
                local_params.ridge_factor_ + 0.002,
                local_params.planning_time_limit_ + 5.0,
                local_params.max_iterations_ + 50);
        }

        auto optimizer = std::make_unique<ChompOptimizer>(&trajectory, planning_scene, req.group_name, &local_params, start_state);
        if (!optimizer->isInitialized())
        {
            RCLCPP_ERROR(LOGGER, "CHOMP Optimizer failed to initialize.");
            res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        optimization_result = optimizer->optimize();
        if (optimization_result || !params.enable_failure_recovery_)
        {
            break;
        }
    }

    if (!optimization_result)
    {
        RCLCPP_ERROR(LOGGER, "CHOMP Optimization failed after recovery attempts.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    // ---------------------- Fill Response Structure ----------------------
    auto result_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(planning_scene->getRobotModel(), req.group_name);
    for (size_t i = 0; i < trajectory.getNumPoints(); ++i)
    {
        auto waypoint_state = std::make_shared<moveit::core::RobotState>(start_state);
        size_t joint_index = 0;
        for (const auto* joint_model : joint_models)
        {
            waypoint_state->setVariablePosition(joint_model->getFirstVariableIndex(), trajectory(i, joint_index++));
        }
        result_trajectory->addSuffixWayPoint(waypoint_state, 0.0);
    }

    res.trajectory_.resize(1);
    res.trajectory_[0] = result_trajectory;
    res.processing_time_ = {std::chrono::duration<double>(std::chrono::system_clock::now() - start_time).count()};
    res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    // Final goal constraint validation
    if (!std::all_of(goal_constraints.joint_constraints.begin(),
                     goal_constraints.joint_constraints.end(),
                     [&result_trajectory, &planning_scene](const auto& joint_constraint) {
                         kinematic_constraints::JointConstraint constraint_checker(planning_scene->getRobotModel());
                         return constraint_checker.configure(joint_constraint) &&
                                constraint_checker.decide(result_trajectory->getLastWayPoint()).satisfied;
                     }))
    {
        RCLCPP_ERROR(LOGGER, "One or more goal constraints are not satisfied.");
        res.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED;
        return false;
    }

    return true;
}


}  // namespace chomp
