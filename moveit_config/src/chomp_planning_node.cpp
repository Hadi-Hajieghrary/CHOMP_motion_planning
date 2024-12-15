#include <algorithm>
#include <map>
#include <open3d/Open3D.h>
#include <Eigen/Dense>


#include <rclcpp/rclcpp.hpp>

// MoveIt 2 includes
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <geometric_shapes/shape_operations.h>

// Messages
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>


#include <tf2_eigen/tf2_eigen.hpp>



// Function to extract trajectory points of the links
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





int main(int argc, char** argv)
{

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("chomp_planning", node_options);

    // --------------------------------------------------------------------------
    // 1. Load the robot model
    // --------------------------------------------------------------------------
    // robot_description parameters should be loaded by your launch file. The RobotModelLoader will look them up.
    // Make sure your node has access to these parameters.

    // Create the RobotModelLoader
    RCLCPP_INFO(node->get_logger(), "\033[1;34mLoading robot model...\033[0m");
    robot_model_loader::RobotModelLoader::Options options;
    options.robot_description_ = "robot_description";
    robot_model_loader::RobotModelLoader robot_model_loader(node, options);
    // Retrive the Model of the Robot
    moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();

    if (!robot_model)
    {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mFailed to load robot model. Ensure robot_description is on param server.\033[0m");
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "\033[1;32mLoading robot model... Done!\033[0m");
    RCLCPP_INFO(node->get_logger(), "\033[1;34mLoaded robot model: %s\033[0m", robot_model->getName().c_str());



    // --------------------------------------------------------------------------
    // 2. Create a PlanningScene
    // --------------------------------------------------------------------------
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);


    // --------------------------------------------------------------------------
    // 3. Add Collision Objects
    // --------------------------------------------------------------------------
    {
        // Create Collision Objects message
        // Set the pose of the Collision Object - In any Frame
        // Proccess it into the planning Scene planning_scene->processCollisionObjectMsg(...);
    }

    // --------------------------------------------------------------------------
    // 4. Set Start and Goal States
    // --------------------------------------------------------------------------
    moveit::core::RobotState& current_state = planning_scene->getCurrentStateNonConst();
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
                                                shoulder_lift_joint,
                                                shoulder_pan_joint,
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
                                            shoulder_lift_joint + 1.5,
                                            shoulder_pan_joint + 0.7,
                                            elbow_joint - 0.4,
                                            wrist_1_joint,
                                            wrist_2_joint,
                                            wrist_3_joint - 0.2
                                        };

    // Create goal constraints from the target joint values
    moveit_msgs::msg::Constraints goal_constraints = kinematic_constraints::constructGoalConstraints(current_state, joint_model_group);

    RCLCPP_INFO(node->get_logger(), "\033[1;32mStart and goal states defined.\033[0m");


    // --------------------------------------------------------------------------
    // 5. Create MotionPlanRequest
    // --------------------------------------------------------------------------
    moveit_msgs::msg::MotionPlanRequest req;
    req.group_name = planning_group;
    req.start_state = current_state_msg;
    req.goal_constraints.push_back(goal_constraints);
    req.allowed_planning_time = 5.0; // 5 seconds of allowed planning time

    RCLCPP_INFO(node->get_logger(), "\033[1;32mMotion plan request created.\033[0m");


    // --------------------------------------------------------------------------
    // 6. Configure and use the CHOMP Planning Pipeline
    // --------------------------------------------------------------------------
    auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "chomp.planning_plugin", "chomp.request_adapters");
    
    RCLCPP_INFO(node->get_logger(), "CHOMP planning pipeline configured.");

    // Run the planner
    planning_interface::MotionPlanResponse res;
    planning_pipeline->generatePlan(planning_scene, req, res);

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "\033[1;31mCHOMP failed to find a plan.\033[0m");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "\033[1;32mCHOMP planning succeeded!\033[0m");

    // --------------------------------------------------------------------------
    // 7. Extract the planned Trajectory
    // --------------------------------------------------------------------------
    const robot_trajectory::RobotTrajectoryPtr& planned_trajectory =  res.trajectory_;
    

    rclcpp::spin(node);

}


