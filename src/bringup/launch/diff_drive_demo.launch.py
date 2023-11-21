from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, AndSubstitution

from os import path
import os

def get_package_lib_directory(package_name):
    return path.join(get_package_prefix(package_name), "lib", package_name)

def append_path(new_path, existing_value):
    if existing_value is None or existing_value == "":
        return new_path
    else:
        return new_path + ":" + existing_value


def generate_launch_description():
    bringup_package = get_package_share_directory("bringup")
    dependency_package_lib_dir = get_package_lib_directory("ros_gz_example_gazebo")

    system_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH")
    os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"] = append_path(dependency_package_lib_dir, system_plugin_path)

    # Set key environment variables
    gz_model_paths_vars = [
        "IGN_GAZEBO_RESOURCE_PATH",
        # "GZ_SIM_RESOURCE_PATH",
        # "GAZEBO_MODEL_PATH",
    ]

    for var in gz_model_paths_vars:
        value = os.environ.get(var)
        if value is None:
            os.environ[var] = path.join(bringup_package, "models")
        else:
            os.environ[var] = path.join(bringup_package, "models") + ":" + value


    # Create the diff_drive_control_node. Because we only create one node and we don't need to resolve any name and
    # namespace conflict, the creation of this node is straightforward.
    print("Creating the diff drive control node")
    diff_drive_control_node = Node(
        package="diff_drive_controller",
        executable="diff_drive_controller_node"
    )

    # Create a robot state publisher based on the robot model
    print("Load the robot model files and create the robot state publisher")
    robot_model_sdf_file = path.join(bringup_package, "models", "diff_drive_ros", "model.sdf")
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
    print("Create the bridge between ROS and Gazebo")
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': path.join(bringup_package, 'config', 'ros_gz_bridge.yml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    print("Create the Gazebo node")
    ros_gz_sim_package = get_package_share_directory('ros_gz_sim')
    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(ros_gz_sim_package, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            bringup_package,
            'worlds',
            'diff_drive_demo_world.sdf'
        ])}.items(),
    )

    return LaunchDescription([
        diff_drive_control_node,
        robot_state_publisher,
        # joint_state_publisher,
        rviz,
        bridge,
        gz_sim_node
    ])
