"""Publish RealSense-like sensor messages from MuJoCo cameras and an IMU broadcaster.

IR images are grayscale renderings, not a simulation of the infrared spectrum.
Depth is ideal MuJoCo geometry, not stereo disparity. Nominal geometry sources:
https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d435.urdf.xacro
https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d435i_imu_modules.urdf.xacro
"""

from copy import deepcopy

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from realsense2_camera_msgs.msg import Extrinsics
from sensor_msgs.msg import CameraInfo, Image, Imu

from simulation_bringup.d435i_geometry import align_depth_to_color, depth_to_uint16


def image_array(message):
    """Read an image respecting its row stride and byte order."""
    if message.encoding == '32FC1':
        dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
        shape, strides = (message.height, message.width), (message.step, 4)
    elif message.encoding == 'rgb8':
        dtype = np.dtype('uint8')
        shape = (message.height, message.width, 3)
        strides = (message.step, 3, 1)
    else:
        raise ValueError(f'Unexpected MuJoCo image encoding: {message.encoding}')
    return np.ndarray(shape, dtype=dtype, buffer=message.data, strides=strides)


def make_image(array, header, encoding):
    """Build a little-endian contiguous image."""
    array = np.ascontiguousarray(array)
    message = Image()
    message.header = deepcopy(header)
    message.height, message.width = array.shape[:2]
    message.encoding = encoding
    message.is_bigendian = False
    message.step = array.strides[0]
    message.data = array.tobytes()
    return message


class D435iBridge(Node):
    """Keep timestamps from the simulator and publish driver-compatible payloads."""

    def __init__(self):
        super().__init__('d435i_bridge', namespace='camera')
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('depth_to_color_translation', [0.015, 0.0, 0.0])
        self.declare_parameter('depth_to_imu_translation', [-0.00552, 0.0051, 0.01174])
        self.declare_parameter('infra_baseline', 0.050)
        self.declare_parameter('publish_legacy_head_topics', True)
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        if not 0 < self.min_depth < self.max_depth <= 65.535:
            raise ValueError('Depth limits must satisfy 0 < min_depth < max_depth <= 65.535m')
        self.translation = self.get_parameter('depth_to_color_translation').value
        self.baseline = self.get_parameter('infra_baseline').value
        self.legacy = self.get_parameter('publish_legacy_head_topics').value
        self.depth_info = None
        self.color_info = None
        self.publishers_by_topic = {}
        self.subscriptions_kept = []
        for stream in ('depth', 'infra1', 'infra2', 'aligned_depth_to_color'):
            self.add_publisher(CameraInfo, f'/camera/{stream}/camera_info')
        for topic in ('depth/image_rect_raw', 'aligned_depth_to_color/image_raw',
                      'infra1/image_rect_raw', 'infra2/image_rect_raw'):
            self.add_publisher(Image, '/camera/' + topic)
        for topic in ('imu', 'gyro/sample', 'accel/sample'):
            self.add_publisher(Imu, '/camera/' + topic)
        if self.legacy:
            self.add_publisher(Image, '/head_camera/color/image_raw')
            self.add_publisher(Image, '/head_camera/depth/image_raw')
            self.add_publisher(CameraInfo, '/head_camera/color/camera_info')
        self.subscribe(CameraInfo, '/camera/_sim/infra1/camera_info', self.on_depth_info)
        self.subscribe(CameraInfo, '/camera/_sim/infra2/camera_info', self.on_right_info)
        self.subscribe(CameraInfo, '/camera/color/camera_info', self.on_color_info)
        self.subscribe(Image, '/camera/_sim/depth_float', self.on_depth)
        self.subscribe(Image, '/camera/_sim/infra1/rgb', lambda msg: self.on_infra(msg, 1))
        self.subscribe(Image, '/camera/_sim/infra2/rgb', lambda msg: self.on_infra(msg, 2))
        self.subscribe(Imu, '/camera/_sim/imu', self.on_imu)
        if self.legacy:
            self.subscribe(Image, '/camera/color/image_raw', self.on_color)
        self.extrinsics_publishers = []
        translations = {
            'color': self.translation,
            'infra1': [0.0, 0.0, 0.0],
            'infra2': [-self.baseline, 0.0, 0.0],
            'gyro': self.get_parameter('depth_to_imu_translation').value,
            'accel': self.get_parameter('depth_to_imu_translation').value,
        }
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for target, translation in translations.items():
            pub = self.create_publisher(Extrinsics, f'/camera/extrinsics/depth_to_{target}', qos)
            extrinsics = Extrinsics()
            extrinsics.rotation = np.eye(3).flatten(order='F').tolist()
            extrinsics.translation = list(translation)
            pub.publish(extrinsics)
            self.extrinsics_publishers.append(pub)
        self.get_logger().info(
            'D435i nominal simulation bridge: geometric depth, grayscale IR, raw gyro/accel; '
            'public IMU orientation is unavailable (covariance[0]=-1).')

    def add_publisher(self, message_type, topic):
        self.publishers_by_topic[topic] = self.create_publisher(message_type, topic, 5)

    def publish(self, topic, message):
        self.publishers_by_topic[topic].publish(message)

    def subscribe(self, message_type, topic, callback):
        self.subscriptions_kept.append(self.create_subscription(
            message_type, topic, callback, qos_profile_sensor_data))

    def on_depth_info(self, message):
        self.depth_info = deepcopy(message)
        self.depth_info.header.frame_id = 'camera_depth_optical_frame'
        self.publish('/camera/depth/camera_info', self.depth_info)
        self.publish('/camera/infra1/camera_info', message)

    def on_right_info(self, message):
        message.p[3] = -message.p[0] * self.baseline
        self.publish('/camera/infra2/camera_info', message)

    def on_color_info(self, message):
        self.color_info = message
        if self.legacy:
            self.publish('/head_camera/color/camera_info', message)

    def on_color(self, message):
        self.publish('/head_camera/color/image_raw', message)

    def on_infra(self, message, index):
        rgb = image_array(message).astype(np.uint16)
        # Integer BT.601 luma, deterministic and within uint16 intermediate range.
        gray = ((77 * rgb[:, :, 0] + 150 * rgb[:, :, 1]
                 + 29 * rgb[:, :, 2]) >> 8).astype(np.uint8)
        self.publish(f'/camera/infra{index}/image_rect_raw',
                     make_image(gray, message.header, 'mono8'))

    def on_depth(self, message):
        depth = image_array(message)
        header = deepcopy(message.header)
        header.frame_id = 'camera_depth_optical_frame'
        raw = depth_to_uint16(depth, self.min_depth, self.max_depth)
        self.publish('/camera/depth/image_rect_raw', make_image(raw, header, '16UC1'))
        if self.depth_info is None or self.color_info is None:
            return
        aligned = align_depth_to_color(
            depth, self.depth_info.k, self.color_info.k,
            (self.color_info.height, self.color_info.width), self.translation,
            self.min_depth, self.max_depth)
        header.frame_id = 'camera_color_optical_frame'
        output = make_image(aligned, header, '16UC1')
        info = deepcopy(self.color_info)
        info.header = deepcopy(header)
        self.publish('/camera/aligned_depth_to_color/image_raw', output)
        self.publish('/camera/aligned_depth_to_color/camera_info', info)
        if self.legacy:
            self.publish('/head_camera/depth/image_raw', output)

    def on_imu(self, message):
        # D435i supplies angular velocity + specific force, not a world-frame
        # orientation. Never leak MuJoCo's ground-truth quaternion to consumers.
        message.orientation.x = message.orientation.y = message.orientation.z = 0.0
        message.orientation.w = 0.0
        message.orientation_covariance = [-1.0] + [0.0] * 8
        message.header.frame_id = 'camera_imu_optical_frame'
        self.publish('/camera/imu', message)
        gyro = deepcopy(message)
        gyro.header.frame_id = 'camera_gyro_optical_frame'
        gyro.linear_acceleration.x = gyro.linear_acceleration.y = gyro.linear_acceleration.z = 0.0
        gyro.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        self.publish('/camera/gyro/sample', gyro)
        accel = deepcopy(message)
        accel.header.frame_id = 'camera_accel_optical_frame'
        accel.angular_velocity.x = accel.angular_velocity.y = accel.angular_velocity.z = 0.0
        accel.angular_velocity_covariance = [-1.0] + [0.0] * 8
        self.publish('/camera/accel/sample', accel)


def main(args=None):
    """Start the sensor bridge."""
    rclpy.init(args=args)
    node = D435iBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
