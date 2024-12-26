import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_param_builder import load_xacro, load_yaml
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Dictionary to hold MoveIt configuration parameters
    moveit_config = {}

    # Load robot description (URDF)
    # Load robot description (URDF)
    name = LaunchConfiguration("name", default="ur5e_manipulator")
    ur_type = LaunchConfiguration("ur_type", default="ur5e")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands", default="true")
    safety_limits = LaunchConfiguration("safety_limits", default="true")
    # General arguments
    description_package = LaunchConfiguration("description_package", default="ur_description")
    description_file = LaunchConfiguration("description_file", default="ur_mocked.urdf.xacro")
    tf_prefix = LaunchConfiguration("tf_prefix", default="")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "name:=",
            name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )

    moveit_config["robot_description"] = ParameterValue(value=robot_description_content, value_type=str)

    # Load semantic robot description (SRDF)
    robot_description_semantic_content = load_xacro(
                                                        Path(get_package_share_directory("moveit_config")) / "config" / "ur5e_manipulator.srdf"
                                                    )
    moveit_config["robot_description_semantic"] = robot_description_semantic_content

    # Load kinematics configuration
    robot_description_kinematics_content = load_yaml(
        Path(get_package_share_directory("moveit_config")) / "config" / "robot_description_kinematics.yaml"
    )
    moveit_config["robot_description_kinematics"] = robot_description_kinematics_content

    # Load planning pipelines
    moveit_config["planning_pipelines"] = ["ompl", "chomp"]
    moveit_config["default_planning_pipeline"] = "ompl"
    moveit_config["ompl"] = load_yaml(Path(get_package_share_directory("moveit_config")) / "config" / "ompl_planning.yaml")
    moveit_config["chomp"] = load_yaml(Path(get_package_share_directory("moveit_config")) / "config" / "chomp_planning.yaml")

    # Load trajectory execution configuration
    trajectory_execution_content = load_yaml(Path(get_package_share_directory("moveit_config")) / "config" / "moveit_controllers.yaml")
    moveit_config.update(trajectory_execution_content)

    # Planning scene monitor configuration
    moveit_config["publish_planning_scene"] = True
    moveit_config["publish_geometry_updates"] = True
    moveit_config["publish_state_updates"] = True
    moveit_config["publish_transforms_updates"] = True
    moveit_config["publish_robot_description"] = True
    moveit_config["publish_robot_description_semantic"] = True

    # Load joint limits
    moveit_config["robot_description_planning"] = load_yaml(
        Path(get_package_share_directory("moveit_config")) / "config" / "joint_limits.yaml"
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_config") + "/rviz" + "/rviz.rviz"
    )   
    rviz_params = [
                        {"robot_description": moveit_config["robot_description"]},
                        {"robot_description_semantic": moveit_config["robot_description_semantic"]},
                        {"robot_description_kinematics": moveit_config["robot_description_kinematics"]},
                        {
                            "planning_pipelines" : moveit_config["planning_pipelines"],
                            "default_planning_pipeline" : moveit_config["default_planning_pipeline"],
                            "ompl": moveit_config["ompl"],
                            "chomp": moveit_config["chomp"]
                        },
                        {'robot_description_planning': moveit_config['robot_description_planning']}
                    ]
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters= rviz_params,
    )


    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
                        {"robot_description": moveit_config["robot_description"]}
                    ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "ur5e_manipulator_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            rviz_node,
        ]
        + load_controllers
    )
