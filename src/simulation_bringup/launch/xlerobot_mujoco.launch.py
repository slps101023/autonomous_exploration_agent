import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # LaunchConfiguration
    # ============================================================

    use_sim_time = LaunchConfiguration("use_sim_time")
    mjcf_path = LaunchConfiguration("mjcf_path")


    # ============================================================
    # Package directories
    # ============================================================

    robot_description_dir = get_package_share_directory(
        "robot_description"
    )

    simulation_bringup_dir = get_package_share_directory(
        "simulation_bringup"
    )


    # ============================================================
    # Paths
    # ============================================================

    urdf_path = os.path.join(
        robot_description_dir,
        "urdf",
        "xlerobot",
        "xlerobot.urdf",
    )

    default_mjcf_path = os.path.join(
        simulation_bringup_dir,
        "model",
        "scene.xml",
    )

    controllers_config_path = os.path.join(
        simulation_bringup_dir,
        "config",
        "controllers.yaml",
    )

    display_launch_path = os.path.join(
        robot_description_dir,
        "launch",
        "display.launch.py",
    )


    # ============================================================
    # Read URDF
    # ============================================================

    with open(urdf_path, "r", encoding="utf-8") as file:
        robot_description = file.read()


    # ============================================================
    # DeclareLaunchArgument
    # ============================================================

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )

    mjcf_path_arg = DeclareLaunchArgument(
        "mjcf_path",
        default_value=default_mjcf_path,
        description="Path to MuJoCo scene.xml",
    )


    # ============================================================
    # mujoco_ros2_control
    # ============================================================

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
                "mujoco_model": mjcf_path,
            },
            controllers_config_path,
        ],
    )


    # ============================================================
    # joint_state_broadcaster
    # ============================================================

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )


    # ============================================================
    # Include display.launch.py
    # ============================================================

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            display_launch_path
        ),
        launch_arguments={
            "model": urdf_path,
            "use_sim_time": use_sim_time,
            "use_gui": "false",
            "use_rviz": "true",
        }.items(),
    )


    # ============================================================
    # LaunchDescription
    # ============================================================

    return LaunchDescription([
        use_sim_time_arg,
        mjcf_path_arg,

        mujoco_ros2_control_node,
        joint_state_broadcaster_node,
        display_launch,
    ])