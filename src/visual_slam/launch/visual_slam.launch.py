"""Launch the existing XLeRobot simulation with head-controlled RGB-D SLAM."""

from datetime import datetime
import math
import os
from pathlib import Path
import shlex
import time
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments are visible with `ros2 launch ... --show-args`.
    use_sim_time = LaunchConfiguration('use_sim_time')
    head_pan = LaunchConfiguration('head_pan')
    head_tilt = LaunchConfiguration('head_tilt')
    start_teleop = LaunchConfiguration('start_teleop')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    frame_id = LaunchConfiguration('frame_id')
    approx_rgbd_sync = LaunchConfiguration('approx_rgbd_sync')
    approx_sync = LaunchConfiguration('approx_sync')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')
    database_path = LaunchConfiguration('database_path')

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('head_pan', default_value='0', description='Head pan angle in radians'),
        DeclareLaunchArgument('head_tilt', default_value='0', description='Head tilt angle in radians'),
        DeclareLaunchArgument('start_teleop', default_value='true'),
        DeclareLaunchArgument('rgb_topic', default_value='/head_camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/head_camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/head_camera/color/camera_info'),
        DeclareLaunchArgument('odom_topic', default_value='/rtabmap/odom'),
        DeclareLaunchArgument('frame_id', default_value='chassis'),
        DeclareLaunchArgument('approx_rgbd_sync', default_value='false'),
        DeclareLaunchArgument('approx_sync', default_value='true'),
        DeclareLaunchArgument('speed', default_value='0.2'),
        DeclareLaunchArgument('turn', default_value='0.5'),
    ]

    simulation_launch = (
        Path(get_package_share_directory('simulation_bringup'))
        / 'launch' / 'xlerobot_mujoco.launch.py'
    )
    rviz_config = (
        Path(get_package_share_directory('rtabmap_launch'))
        / 'launch' / 'config' / 'rgbd.rviz'
    )
    map_dir = Path('/home/lihsun/autonomous_exploration_agent/src/visual_slam/maps')

    def configure_map(context):
        pan = float(head_pan.perform(context))
        tilt = float(head_tilt.perform(context))
        if not math.isfinite(pan) or not math.isfinite(tilt):
            raise RuntimeError('head_pan and head_tilt must be finite numbers')
        if not -3.2 <= pan <= 3.2 or not -0.76 <= tilt <= 1.45:
            raise RuntimeError('head_pan must be in [-3.2, 3.2] and head_tilt in [-0.76, 1.45] radians')

        def angle_text(value):
            return f'{value:.6f}'.rstrip('0').rstrip('.') if value else '0'

        map_dir.mkdir(parents=True, exist_ok=True)
        while True:
            stamp = datetime.now(ZoneInfo('Asia/Taipei')).strftime('%Y%m%d_%H%M%S')
            path = map_dir / (
                f'visual_slam_pan{angle_text(pan)}_tilt{angle_text(tilt)}_{stamp}.db'
            )
            if not path.exists():
                break
            time.sleep(0.2)
        return [
            SetLaunchConfiguration('database_path', str(path)),
            LogInfo(msg=f'RTAB-Map database: {path}'),
            LogInfo(msg=f'Head pan/tilt: {pan}, {tilt} rad'),
        ]

    # Node receives its executable and ROS arguments after this terminal prefix.
    terminal_script = '\n'.join((
        'set -e',
        'source /opt/ros/lyrical/setup.bash',
        'source /home/lihsun/autonomous_exploration_agent/install/setup.bash',
        '"$@" </dev/tty &',
        'teleop_pid=$!',
        f'while kill -0 {os.getpid()} 2>/dev/null && kill -0 "$teleop_pid" 2>/dev/null; do sleep 0.5; done',
        'pkill -KILL -P "$teleop_pid" 2>/dev/null || true',
        'kill -KILL "$teleop_pid" 2>/dev/null || true',
        'wait "$teleop_pid" 2>/dev/null || true',
    ))
    terminal_prefix = (
        'gnome-terminal --wait --title=Robot_keyboard_control -- bash -lc '
        + shlex.quote(terminal_script) + ' _'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulation_launch)),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    head_controller = Node(
        package='visual_slam', executable='head_pose_commander',
        arguments=[head_pan, head_tilt], output='screen',
    )
    rgbd_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync',
        name='rgbd_sync', namespace='rtabmap', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'approx_sync': ParameterValue(approx_rgbd_sync, value_type=bool),
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
        ],
    )
    visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'frame_id': frame_id,
            'odom_frame_id': 'visual_odom',
            'publish_tf': True,
            'subscribe_rgbd': True,
        }],
        remappings=[
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('odom', '/rtabmap/odom'),
        ],
    )
    slam = Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', namespace='rtabmap', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_rgbd': True,
            'frame_id': frame_id,
            'map_frame_id': 'map',
            'publish_tf': True,
            'approx_sync': ParameterValue(approx_sync, value_type=bool),
            'database_path': ParameterValue(database_path, value_type=str),
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',     
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('odom', odom_topic),
        ],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', str(rviz_config)], output='screen',
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )
    point_cloud = Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb', output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'decimation': 4,
            'voxel_size': 0.0,
            'approx_sync': ParameterValue(approx_sync, value_type=bool),
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('rgbd_image', '/rtabmap/rgbd_image'),
            ('cloud', '/voxel_cloud'),
        ],
    )
    tele_key = Node(
        package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
        name='visual_slam_keyboard', prefix=terminal_prefix, output='screen',
        condition=IfCondition(start_teleop),
        parameters=[{
            'stamped': True,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'frame_id': frame_id,
            'speed': ParameterValue(speed, value_type=float),
            'turn': ParameterValue(turn, value_type=float),
        }],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel')],
    )

    return LaunchDescription([
        *arguments,
        OpaqueFunction(function=configure_map),
        simulation,
        head_controller,
        rgbd_sync,
        visual_odometry,
        slam,
        rviz,
        point_cloud,
        tele_key,
    ])
