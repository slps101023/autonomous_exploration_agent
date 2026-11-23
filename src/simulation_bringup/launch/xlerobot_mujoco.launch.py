import os
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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

    camera_config_path = os.path.join(
        simulation_bringup_dir,
        "config",
        "mujoco_camera.yaml",
    )

    publish_image_transports_arg = DeclareLaunchArgument(
        "publish_image_transports", default_value="true",
        description="Publish compatible compressed/theora image transports.",
    )

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

    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run MuJoCo without its GUI (camera rendering still uses OpenGL/EGL).",
    )


    # ============================================================
    # mujoco_ros2_control
    # ============================================================

    def create_controller_manager(context):
        # headless is a ros2_control hardware parameter, not a ROS node parameter.
        # Both nodes need the same URDF: controller_manager receives the latched
        # robot_description topic from robot_state_publisher on current ROS 2.
        root = ET.parse(urdf_path).getroot()
        hardware = root.find('ros2_control/hardware')
        parameter = hardware.find("param[@name='headless']")
        if parameter is None:
            parameter = ET.SubElement(hardware, 'param', name='headless')
        parameter.text = LaunchConfiguration('headless').perform(context)
        robot_description = ET.tostring(root, encoding='unicode')
        return [Node(
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
                camera_config_path,
            ],
        ), Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{
                "robot_description": robot_description, "use_sim_time": use_sim_time,
            }],
        )]

    # ============================================================
    # diff_drive_controller
    # ============================================================

    diff_drive_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
        output="screen",
    )


    # ============================================================
    # head_position_controller
    # ============================================================

    head_position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "head_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
        output="screen",
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


    chassis_pose_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "chassis_pose_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_config_path,
        ],
        output="screen",
    )

    camera_imu_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "camera_imu_broadcaster", "--controller-manager", "/controller_manager",
            "--param-file", controllers_config_path,
            "--controller-ros-args", "--remap ~/imu:=/camera/_sim/imu",
        ],
        output="screen",
    )
    camera_bridge = Node(
        package="simulation_bringup", executable="d435i_bridge",
        name="d435i_bridge", namespace="camera", output="screen",
        parameters=[camera_config_path, {"use_sim_time": use_sim_time}],
    )
    image_transports = []
    # A specific output transport avoids republishing raw onto its input topic.
    # Theora is only useful for RGB/mono images, compressedDepth for depth images.
    for stream, topic, transports in (
        ("color", "/camera/color/image_raw", ("compressed", "theora")),
        ("depth", "/camera/depth/image_rect_raw", ("compressed", "compressedDepth")),
        ("aligned_depth", "/camera/aligned_depth_to_color/image_raw",
         ("compressed", "compressedDepth")),
        ("infra1", "/camera/infra1/image_rect_raw", ("compressed",)),
        ("infra2", "/camera/infra2/image_rect_raw", ("compressed",)),
    ):
        for transport in transports:
            image_transports.append(Node(
                package="image_transport", executable="republish",
                name=f"{stream}_{transport}", namespace="camera", output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "in_transport": "raw", "out_transport": transport,
                }],
                remappings=[("in", topic), ("out", topic)],
                condition=IfCondition(LaunchConfiguration("publish_image_transports")),
            ))

    # ============================================================
    # LaunchDescription
    # ============================================================

    return LaunchDescription([
        use_sim_time_arg,
        mjcf_path_arg,
        headless_arg,
        publish_image_transports_arg,
        OpaqueFunction(function=create_controller_manager),
        joint_state_broadcaster_node,
        diff_drive_controller_node,
        head_position_controller_node,
        chassis_pose_broadcaster_node,
        camera_imu_broadcaster,
        camera_bridge,
        *image_transports,
    ])
