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
    ld = LaunchDescription()

    # Load robot description (URDF)
    name = LaunchConfiguration("name", default="ur5e_manipulator")
    ur_type = LaunchConfiguration("ur_type", default="ur5e")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands", default="true")
    safety_limits = LaunchConfiguration("safety_limits", default="true")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin", default="0.15")
    safety_k_position = LaunchConfiguration("safety_k_position", default="20",)
    # General arguments
    description_package = LaunchConfiguration("description_package", default="ur_description")
    description_file = LaunchConfiguration("description_file", default="ur.urdf.xacro")
    tf_prefix = LaunchConfiguration("tf_prefix", default="")

    robot_description_content = Command(
                                        [
                                            PathJoinSubstitution([FindExecutable(name="xacro")]),
                                            " ",
                                            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
                                            " ",
                                            "use_fake_hardware:=",
                                            use_fake_hardware,
                                            " ",
                                            "fake_sensor_commands:=",
                                            fake_sensor_commands,
                                            " ",
                                            "safety_limits:=",
                                            safety_limits,
                                            " ",
                                            "safety_pos_margin:=",
                                            safety_pos_margin,
                                            " ",
                                            "safety_k_position:=",
                                            safety_k_position,
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
 
    # Load semantic robot description (SRDF)
    robot_description_semantic_content = load_xacro(
        Path(get_package_share_directory("moveit_config")) / "config" / "ur5e_manipulator.srdf"
    )

    # Load kinematics configuration
    robot_description_kinematics_content = load_yaml(
        Path(get_package_share_directory("moveit_config")) / "config" / "robot_description_kinematics.yaml"
    )

    # Load kinematics configuration
    robot_initial_position_content = load_yaml(
        Path(get_package_share_directory("moveit_config")) / "config" / "initial_position.yaml"
    )

    # Load CHOMP configuration
    chomp_planning_content = load_yaml(
        Path(get_package_share_directory("moveit_config")) / "config" / "chomp_planning.yaml"
    )

    # Start the actual move_group node/action server
    chomp_planning_node = Node(
                                    package="moveit_config",
                                    executable="chomp_planning",
                                    output="screen",
                                    parameters=[
                                                {
                                                    "robot_description": robot_description_content,
                                                    "robot_description_semantic": robot_description_semantic_content,
                                                    "robot_description_kinematics": robot_description_kinematics_content,
                                                    "robot_initial_position": robot_initial_position_content,
                                                    "chomp": chomp_planning_content
                                                }
                                                ],
                                )

    ld.add_action(chomp_planning_node)


    return ld
