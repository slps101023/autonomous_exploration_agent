"""Report whether the ROS inputs needed for RGB-D SLAM are arriving."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class InputStatus(Node):
    """Watch the camera and odometry topics before starting SLAM."""

    def __init__(self):
        super().__init__('visual_slam_input_status')
        defaults = {
            'rgb_topic': '/head_camera/color/image_raw',
            'depth_topic': '/head_camera/depth/image_raw',
            'camera_info_topic': '/head_camera/color/camera_info',
            'odom_topic': '/odom',
        }
        self._last_seen = {name: None for name in defaults}
        self._last_report = None
        message_types = {
            'rgb_topic': Image,
            'depth_topic': Image,
            'camera_info_topic': CameraInfo,
            'odom_topic': Odometry,
        }
        for name, default in defaults.items():
            topic = self.declare_parameter(name, default).value
            self.create_subscription(
                message_types[name],
                topic,
                lambda _msg, key=name: self._mark_seen(key),
                qos_profile_sensor_data,
            )
            self.get_logger().info(f'Watching {name}: {topic}')
        self.create_timer(5.0, self._report)

    def _mark_seen(self, name):
        self._last_seen[name] = self.get_clock().now()

    def _report(self):
        now = self.get_clock().now()
        missing = [
            name for name, last_seen in self._last_seen.items()
            if last_seen is None or (now - last_seen).nanoseconds > 5_000_000_000
        ]
        report = tuple(missing)
        if report != self._last_report:
            if missing:
                self.get_logger().warning('No recent data: ' + ', '.join(missing))
            else:
                self.get_logger().info('All RGB-D and odometry inputs are arriving.')
            self._last_report = report


def main(args=None):
    """Run the input status node."""
    rclpy.init(args=args)
    node = InputStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
