import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'urdf',
            'xlerobot',
            'xlerobot.urdf',
        ),
        description='Absolute path to the XLeRobot URDF or Xacro file.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock when true.',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='true',
        description='Start joint_state_publisher_gui.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz2.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('robot_description'),
            'rviz',
            'xlerobot.rviz',
        ),
        description='Path to the RViz configuration file.',
    )

    # Find package resources
    robot_description_package_path = get_package_share_directory('robot_description')

    # Build the robot_description parameter from URDF/Xacro
    robot_description = Command(['xacro ', model])
    robot_description_param = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time,
    }

    # Nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description_param],
        condition=IfCondition(use_gui),
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        use_gui_arg,
        use_rviz_arg,
        rviz_config_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])
