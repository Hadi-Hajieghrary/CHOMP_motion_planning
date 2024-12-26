from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

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
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        name="joint_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        #joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
