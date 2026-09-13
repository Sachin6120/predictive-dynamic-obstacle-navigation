#!/usr/bin/env python3
"""Deterministic back-and-forth driver for the Stage-4B moving obstacle.

Drives the `dynamic_obstacle` model (see models/dynamic_obstacle.sdf) along a
straight line by commanding a constant-magnitude velocity through Gazebo's
VelocityControl plugin, reversing direction when the model's own ground-truth
odometry crosses configured bounds. This is a simulation-support helper, not
part of the tracker under test: it exists only to produce a moving obstacle
with a known, reproducible ground-truth trajectory for validation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class ObstacleMover(Node):

    def __init__(self):
        super().__init__('obstacle_mover')

        self.declare_parameter('odom_topic', '/model/dynamic_obstacle/odometry')
        self.declare_parameter('cmd_vel_topic', '/model/dynamic_obstacle/cmd_vel')
        self.declare_parameter('axis', 'axis_y')      # 'axis_x' or 'axis_y'
        self.declare_parameter('speed', 0.25)         # m/s
        self.declare_parameter('bound_min', -1.5)
        self.declare_parameter('bound_max', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.axis = self.get_parameter('axis').value
        self.speed = float(self.get_parameter('speed').value)
        self.bound_min = float(self.get_parameter('bound_min').value)
        self.bound_max = float(self.get_parameter('bound_max').value)

        self.direction = 1.0
        self.have_odom = False

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_callback, 10)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(period, self.publish_cmd)

        self.get_logger().info(
            f"obstacle_mover: axis={self.axis} speed={self.speed} "
            f"bounds=[{self.bound_min}, {self.bound_max}]")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        value = pos.y if self.axis == 'axis_y' else pos.x

        if not self.have_odom:
            self.have_odom = True

        if value >= self.bound_max and self.direction > 0.0:
            self.direction = -1.0
        elif value <= self.bound_min and self.direction < 0.0:
            self.direction = 1.0

    def publish_cmd(self):
        twist = Twist()
        value = self.direction * self.speed
        if self.axis == 'axis_y':
            twist.linear.y = value
        else:
            twist.linear.x = value
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
