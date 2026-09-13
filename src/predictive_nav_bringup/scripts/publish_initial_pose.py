#!/usr/bin/env python3
"""One-shot, subscription-gated /initialpose publisher.

Root cause being fixed (see Stage-4A handoff): nav2_bringup's
lifecycle_manager_navigation gives up bringing up the navigation servers if
`base_link -> map` TF does not appear within its internal bond/activation
timeout. That transform only exists after AMCL processes an initial pose. If
a human publishes /initialpose "whenever they get around to it", the timing
is unreliable and bringup can silently fail.

This node removes the human from that loop: it blocks until AMCL's
subscription to /initialpose is actually present (so the message is
guaranteed to be delivered, not dropped because nobody was listening yet),
publishes exactly once with the configured spawn pose, and exits. It is
launched as a normal bringup action, so every run gets the same timing.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('x', -2.0)
        self.declare_parameter('y', -0.5)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('xy_covariance', 0.25)
        self.declare_parameter('yaw_covariance', 0.0685)
        self.declare_parameter('wait_timeout_sec', 60.0)

        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

    def wait_for_subscriber(self, timeout_sec: float) -> bool:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            if self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pub.get_subscription_count() > 0

    def publish_once(self):
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        frame_id = self.get_parameter('frame_id').value
        xy_cov = float(self.get_parameter('xy_covariance').value)
        yaw_cov = float(self.get_parameter('yaw_covariance').value)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        cov[0] = xy_cov          # x
        cov[7] = xy_cov          # y
        cov[35] = yaw_cov        # yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(
            f"Published /initialpose (x={x}, y={y}, yaw={yaw}) to AMCL")


def main():
    rclpy.init()
    node = InitialPosePublisher()
    timeout = float(node.get_parameter('wait_timeout_sec').value)

    node.get_logger().info("Waiting for AMCL to subscribe to /initialpose...")
    if not node.wait_for_subscriber(timeout):
        node.get_logger().error(
            f"No subscriber appeared on /initialpose within {timeout}s; "
            "AMCL may not be running. Not publishing.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.publish_once()
    # Give the message a moment to actually go out before the process exits.
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
