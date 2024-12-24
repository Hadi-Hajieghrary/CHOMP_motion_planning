#include <algorithm>
#include <map>
#include "sstream"
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

// MoveIt 2 includes
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>


// Messages
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

// CHOMP includes
#include "my_chomp_motion_planner/chomp_parameters.hpp"
#include "my_chomp_motion_planner/chomp_planner.hpp"




std::map<std::string, std::vector<Eigen::Vector3d>> extractTrajectoryPoints(robot_trajectory::RobotTrajectoryPtr trajectory) {
    std::map<std::string, std::vector<Eigen::Vector3d>> result;
    const std::vector<std::string>& link_names = trajectory->getRobotModel()->getLinkModelNames();
    for(const auto& link_name: link_names){
        std::vector<Eigen::Vector3d> points;
        for (size_t i = 0; i < trajectory->getWayPointCount(); ++i) {
            const moveit::core::RobotState& state = trajectory->getWayPoint(i);
            Eigen::Vector3d position = state.getGlobalLinkTransform(link_name.c_str()).translation();
            points.push_back(position);
        }
        result[link_name] = points;
    }
    return result;
}

int main(int argc, char** argv) {

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("chomp_planning", node_options);

    // --------------------------------------------------------------------------
    // 1. Load the robot model
    // --------------------------------------------------------------------------
    RCLCPP_INFO(node->get_logger(), "\033[1;34mLoading robot model...\033[0m");
    robot_model_loader::RobotModelLoader::Options options;
    options.robot_description_ = "robot_description";
    robot_model_loader::RobotModelLoader robot_model_loader(node, options);
    moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();

    if (!robot_model) {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mFailed to load robot model. Ensure robot_description is on param server.\033[0m");
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "\033[1;32mLoading robot model... Done!\033[0m");
    RCLCPP_INFO(node->get_logger(), "\033[1;34mLoaded robot model: %s\033[0m", robot_model->getName().c_str());

    // --------------------------------------------------------------------------
    // 2. Create a PlanningScene
    // --------------------------------------------------------------------------
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    // Force the active collision detector to "HYBRID"
    std::shared_ptr<collision_detection::CollisionDetectorAllocator> collision_detector_aloc = collision_detection::CollisionDetectorAllocatorHybrid::create();
    if (!!collision_detector_aloc)
    {
        planning_scene->allocateCollisionDetector(collision_detector_aloc);
        RCLCPP_INFO(node->get_logger(), "\033[1;34mSuccessfully allocated HYBRID collision detector.\033[0m");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mFailed to allocate the HYBRID collision detector. Make sure the plugin is available.\033[0m");
    }
    // --------------------------------------------------------------------------
    // 3. Add Collision Objects (Placeholder)
    // --------------------------------------------------------------------------

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = robot_model->getModelFrame();
    collision_object.id = "obstacle_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.5;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.5;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene->processCollisionObjectMsg(collision_object);

    auto collision_world = planning_scene->getCollisionEnv()->getWorld();
    if (collision_world->getObjectIds().size() == 0) {
        RCLCPP_WARN(node->get_logger(), "No collision objects in the planning scene!");
    }

    auto collision_detector = planning_scene->getCollisionDetectorName();
    RCLCPP_INFO(node->get_logger(), "Active collision detector is: %s", collision_detector.c_str());

    // --------------------------------------------------------------------------
    // 4. Set Start and Goal States
    // --------------------------------------------------------------------------
    moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
    current_state.updateCollisionBodyTransforms();
    std::string planning_group = "ur5e_manipulator";  // Defined in robot's SRDF
    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup(planning_group);

    double shoulder_lift_joint{0.0};
    double shoulder_pan_joint{0.0};
    double elbow_joint{0.0};
    double wrist_1_joint{0.0};
    double wrist_2_joint{0.0};
    double wrist_3_joint{0.0};

    if (node->has_parameter("robot_initial_position.shoulder_pan_joint")) {
        shoulder_pan_joint = node->get_parameter("robot_initial_position.shoulder_pan_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.shoulder_pan_joint not set, using default value.\033[0m");
    }
    if (node->has_parameter("robot_initial_position.shoulder_lift_joint")) {
        shoulder_lift_joint = node->get_parameter("robot_initial_position.shoulder_lift_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.shoulder_lift_joint not set, using default value.\033[0m");
    }
    if (node->has_parameter("robot_initial_position.elbow_joint")) {
        elbow_joint = node->get_parameter("robot_initial_position.elbow_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.elbow_joint not set, using default value.\033[0m");
    }
    if (node->has_parameter("robot_initial_position.wrist_1_joint")) {
        wrist_1_joint = node->get_parameter("robot_initial_position.wrist_1_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.wrist_1_joint not set, using default value.\033[0m");
    }
    if (node->has_parameter("robot_initial_position.wrist_2_joint")) {
        wrist_2_joint = node->get_parameter("robot_initial_position.wrist_2_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.wrist_2_joint not set, using default value.\033[0m");
    }
    if (node->has_parameter("robot_initial_position.wrist_3_joint")) {
        wrist_3_joint = node->get_parameter("robot_initial_position.wrist_3_joint").as_double();
    }else{
        RCLCPP_WARN(node->get_logger(), "\033[1;33mParameter robot_initial_position.wrist_3_joint not set, using default value.\033[0m");
    }

    if (!joint_model_group)
    {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mJoint model group '%s' not found. Check SRDF.\033[0m", planning_group.c_str());
        return -1;
    }
    
    std::vector<double> start_joint_values {
                                                shoulder_pan_joint,
                                                shoulder_lift_joint,
                                                elbow_joint,
                                                wrist_1_joint,
                                                wrist_2_joint,
                                                wrist_3_joint
                                            };

    current_state.setJointGroupPositions(joint_model_group, start_joint_values);
    // Convert start state to a RobotState message
    moveit_msgs::msg::RobotState current_state_msg;
    moveit::core::robotStateToRobotStateMsg(current_state, current_state_msg);

    // Goal state joint values
    std::vector<double> goal_joint_values{
                                            shoulder_pan_joint + 0.7,
                                            shoulder_lift_joint - 1.5,
                                            elbow_joint - 0.4,
                                            wrist_1_joint,
                                            wrist_2_joint,
                                            wrist_3_joint - 0.2
                                        };
    moveit::core::RobotState goal_state(robot_model);
    goal_state.setJointGroupPositions(joint_model_group, goal_joint_values);
    // Create goal constraints from the target joint values
    moveit_msgs::msg::Constraints goal_constraints = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    RCLCPP_INFO(node->get_logger(), "\033[1;32mStart and goal states defined.\033[0m");

    // --------------------------------------------------------------------------
    // 5. Create MotionPlanRequest
    // --------------------------------------------------------------------------
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name = planning_group;
    req.start_state = current_state_msg;
    req.goal_constraints.push_back(goal_constraints);
    req.allowed_planning_time = 5.0;

    RCLCPP_INFO(node->get_logger(), "\033[1;32mMotion plan request created.\033[0m");

    // --------------------------------------------------------------------------
    // 6. Configure and Use CHOMP Planner Directly
    // --------------------------------------------------------------------------
    chomp::ChompParameters params;
    params.planning_time_limit_ = 10.0;
    params.max_iterations_ = 200;
    params.max_iterations_after_collision_free_ = 5;
    params.smoothness_cost_weight_ = 0.1;
    params.obstacle_cost_weight_ = 1.0;
    params.learning_rate_ = 0.01;
    params.smoothness_cost_velocity_ = 0.0;
    params.smoothness_cost_acceleration_ = 1.0;
    params.smoothness_cost_jerk_ = 0.0;
    params.use_stochastic_descent_ = true;
    params.ridge_factor_ = 0.01;
    params.use_pseudo_inverse_ = false;
    // params.pseudo_inverse_ridge_factor_ = 1e-4;
    params.joint_update_limit_ = 0.1;
    params.min_clearance_ = 0.2;
    params.collision_threshold_ = 0.07;
    params.filter_mode_ = false;
    params.trajectory_initialization_method_ = "quintic-spline";
    params.enable_failure_recovery_ = true;
    params.max_recovery_attempts_ = 5;


    chomp::ChompPlanner chomp_planner;
    planning_interface::MotionPlanDetailedResponse res;

    if (!chomp_planner.solve(planning_scene, req, params, res)) {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mCHOMP failed to find a plan.\033[0m");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "\033[1;32mCHOMP planning succeeded!\033[0m");

    // --------------------------------------------------------------------------
    // 7. Extract the planned Trajectory
    // --------------------------------------------------------------------------
    const std::vector<robot_trajectory::RobotTrajectoryPtr>& planned_trajectories = res.trajectory_;

    // Print Out the Trajectory
    {
        std::ostringstream oss;
        RCLCPP_INFO(node->get_logger(), "\033[1;33mPrinting the Trajectory!!\033[0m");
        for(size_t i{0}; i != planned_trajectories.size(); ++i){
            RCLCPP_INFO(node->get_logger(), "\033[1;33mPrinting the Trajectory No. %ld!!\033[0m", i);
            size_t waypoint_count = planned_trajectories[i]->getWayPointCount();
            // Print Out the Joint Values along the Trajectory
            RCLCPP_INFO(node->get_logger(), "Trajectory Group: %s", planned_trajectories[i]->getGroupName().c_str());
            RCLCPP_INFO(node->get_logger(), "Robot Model:     %s", planned_trajectories[i]->getRobotModel()->getName().c_str());
            RCLCPP_INFO(node->get_logger(), "Waypoint count:  %ld", waypoint_count);

            // Loop through all waypoints
            for (std::size_t p = 0; p != waypoint_count; ++p)
            {
                // Time from start for the i-th waypoint
                double time_from_start = planned_trajectories[i]->getWayPointDurationFromStart(p);
                // Retrieve the i-th RobotState
                const moveit::core::RobotState& waypoint_state = planned_trajectories[i]->getWayPoint(p);
                // Retrieve the names of all joints in the RobotState
                const std::vector<std::string>& joint_names = waypoint_state.getVariableNames();
                // Prepare a stream for printing joint positions
                std::ostringstream oss;
                oss << "Waypoint " << p 
                    << " (t=" << time_from_start << "s): [";

                // Append each joint name and its position
                for (std::size_t j = 0; j < joint_names.size(); ++j)
                {
                // Get the position of this joint
                double position = waypoint_state.getVariablePosition(joint_names[j]);
                oss << joint_names[j] << "=" << position;
                if (j + 1 < joint_names.size())
                    oss << ", ";
                }
                oss << "]";

                // Log the line
                RCLCPP_INFO(node->get_logger(), "\033[0m%s\033[0m", oss.str().c_str());
            }

        }

    }

    rclcpp::spin(node);
}
