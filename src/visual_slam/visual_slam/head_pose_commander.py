"""Set the simulated head joints once their forward command controller is active."""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class HeadPoseCommander(Node):
    def __init__(self, pan: float, tilt: float):
        super().__init__('head_pose_commander')
        self.target = (pan, tilt)
        self.client = self.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/head_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(0.2, self.on_timer)
        self.started = time.monotonic()
        self.pending = None
        self.active = False
        self.sent = False
        self.matches = 0
        self.done = False

    def on_joints(self, message: JointState):
        if not self.sent:
            return
        positions = dict(zip(message.name, message.position))
        if not all(name in positions for name in ('head_pan_joint', 'head_tilt_joint')):
            return
        actual = (positions['head_pan_joint'], positions['head_tilt_joint'])
        self.matches = self.matches + 1 if all(
            abs(value - target) < 0.02
            for value, target in zip(actual, self.target)
        ) else 0
        if self.matches >= 3:
            self.get_logger().info(
                f'Head pose reached: pan={actual[0]:.4f}, tilt={actual[1]:.4f} rad')
            self.done = True

    def on_timer(self):
        if self.done:
            return
        if time.monotonic() - self.started > 45:
            self.get_logger().error('Timed out waiting for active head controller or joint feedback')
            self.done = True
            self.failed = True
            return
        if not self.active:
            if self.pending is not None and self.pending.done():
                result = self.pending.result()
                self.pending = None
                for controller in result.controller:
                    if controller.name == 'head_position_controller':
                        if controller.state == 'active':
                            if controller.type != 'forward_command_controller/ForwardCommandController':
                                self.get_logger().error(f'Unexpected head controller type: {controller.type}')
                                self.failed = True
                                self.done = True
                                return
                            self.active = True
                            self.get_logger().info('head_position_controller is active')
                        break
            if not self.active and self.pending is None and self.client.service_is_ready():
                self.pending = self.client.call_async(ListControllers.Request())
            return
        if self.publisher.get_subscription_count() > 0:
            self.publisher.publish(Float64MultiArray(data=list(self.target)))
            self.sent = True


def main():
    parser = argparse.ArgumentParser(description='Command head pan and tilt in radians')
    parser.add_argument('pan', type=float)
    parser.add_argument('tilt', type=float)
    arguments, _ = parser.parse_known_args()
    rclpy.init()
    node = HeadPoseCommander(arguments.pan, arguments.tilt)
    node.failed = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 1 if node.failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
