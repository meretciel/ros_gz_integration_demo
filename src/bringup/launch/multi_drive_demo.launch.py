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


def read_robot_description(model_file_path):
    with open(model_file_path, "r") as f:
        return f.read()

def generate_launch_description():
    """
    Compared to the diff_drive_demo.launch.py file, this launch configuration does not contain the control node. To
    control the diff drive, we need to launch a teleop_twist_keyboard using the following command:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap rostopic://cmd_vel:=diff_drive_cmd
    """
    bringup_package = get_package_share_directory("bringup")
    dependency_package_lib_dir = get_package_lib_directory("gz_plugin_dependencies")
    world_file_arg = DeclareLaunchArgument("world", default_value="diff_drive_demo_world.sdf")
    world_file_value = LaunchConfiguration("world")

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

    # Create a robot state publisher based on the robot model
    print("Load the robot model files and create the robot state publisher")
    multi_drive_description = read_robot_description(path.join(bringup_package, "models", "diff_drive_ros", "multi_drive_models.sdf"))
    description_1 = read_robot_description(path.join(bringup_package, "models", "diff_drive_ros", "model_drive_1.sdf"))
    description_2 = read_robot_description(path.join(bringup_package, "models", "diff_drive_ros", "model_drive_2.sdf"))
    robot_description = read_robot_description(path.join(bringup_package, "models", "diff_drive", "model.sdf"))
    print("checkpoint")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="my_robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ]
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
            'config_file': path.join(bringup_package, 'config', 'multi_drive_demo_ros_gz_bridge.yml'),
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
            world_file_value,
        ])}.items(),
    )

    diff_drive_control_node_1 = Node(
        name="drive_1",
        package="diff_drive_controller",
        executable="diff_drive_controller_node"
    )
    diff_drive_control_node_2 = Node(
        name="drive_2",
        package="diff_drive_controller",
        executable="diff_drive_controller_node"
    )

    return LaunchDescription([
            world_file_arg,
            robot_state_publisher,
            rviz,
            bridge,
            gz_sim_node,
            diff_drive_control_node_1,
            diff_drive_control_node_2
        ])
