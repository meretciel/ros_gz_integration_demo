from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution

from os import path

def generate_launch_description():
    bringup_package = get_package_share_directory("bringup")

    # Create the diff_drive_control_node. Because we only create one node and we don't need to resolve any name and
    # namespace conflict, the creation of this node is straightforward.
    print("Creating the diff drive control node")
    diff_drive_control_node = Node(
        package="diff_drive_controller",
        executable="diff_drive_controller_node"
    )

    # Create a robot state publisher based on the robot model
    print("Load the robot model files and create the robot state publisher")
    robot_model_sdf_file = path.join(bringup_package, "models", "diff_drive", "model.sdf")
    with open(robot_model_sdf_file, "r") as f:
        robot_model_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="my_robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_model_description},
            {"frame_prefix": "diff_drive"}
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[robot_model_sdf_file],
        output=['screen']
    )

    # Create the rviz node
    print("Create the rviz node.")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # Gazebo Section

    return LaunchDescription([
        diff_drive_control_node,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
