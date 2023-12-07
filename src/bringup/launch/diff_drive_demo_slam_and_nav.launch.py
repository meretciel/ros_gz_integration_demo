from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from os import path
import os

def get_package_lib_directory(package_name):
    return path.join(get_package_prefix(package_name), "lib", package_name)

def append_path(new_path, existing_value):
    if existing_value is None or existing_value == "":
        return new_path
    else:
        return new_path + ":" + existing_value

def define_argument(arg_name: str, default_value: str | None = None):
    arg = DeclareLaunchArgument(arg_name, default_value=default_value)
    value = LaunchConfiguration(arg_name)
    return arg, value


def generate_launch_description():
    """
    Compared to the diff_drive_demo.launch.py file, this launch configuration does not contain the control node. To
    control the diff drive, we need to launch a teleop_twist_keyboard using the following command:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap rostopic://cmd_vel:=diff_drive_cmd
    """
    bringup_package = get_package_share_directory("bringup")
    dependency_package_lib_dir = get_package_lib_directory("gz_plugin_dependencies")

    # Define the launch file arguments
    world_file_arg = DeclareLaunchArgument("world", default_value="diff_drive_demo_world.sdf")
    world_file_value = LaunchConfiguration("world")

    gui_config_arg = DeclareLaunchArgument("gui_config", default_value="slam_and_nav_demo_gui.config")
    gui_config_value = LaunchConfiguration("gui_config")

    disable_slam_arg = DeclareLaunchArgument("disable_slam", default_value="false")
    disable_slam_value = LaunchConfiguration("disable_slam")

    rviz_config_arg, rviz_config_value = define_argument("rviz_config", default_value="slam_and_nav_v1.rviz")


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
    robot_model_sdf_file = path.join(bringup_package, "models", "diff_drive_slam_and_nav", "model.sdf")
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

    # Create the rviz node
    print("Create the rviz node.")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", PathJoinSubstitution([bringup_package, "config", rviz_config_value])]
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
    world_file = PathJoinSubstitution([bringup_package, 'worlds', world_file_value])
    gui_config = PathJoinSubstitution([bringup_package, 'config', gui_config_value])
    # gz_args = " ".join([world_file, "--gui-config", gui_config])
    # print(f"Resolved gz_args: {gz_args}")

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(ros_gz_sim_package, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world_file, " --gui-config ", gui_config]}.items(),
    )

    components_to_launch = [
        world_file_arg,
        gui_config_arg,
        disable_slam_arg,
        robot_state_publisher,
        rviz,
        bridge,
        gz_sim_node,
    ]


    print("Launch the slam toolbox")
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        condition=UnlessCondition(disable_slam_value),
        launch_arguments={
            'use_sim_time': "True",
            'slam_params_file': PathJoinSubstitution([bringup_package, "config", "mapper_params_online_async.yaml"])
        }.items(),
    )

    return LaunchDescription([
        world_file_arg,
        gui_config_arg,
        disable_slam_arg,
        rviz_config_arg,
        robot_state_publisher,
        rviz,
        bridge,
        gz_sim_node,
        slam_toolbox
    ])
