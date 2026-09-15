#!/usr/bin/env python3
"""Stage-4G8 sensor-noise injector: /scan -> /scan_noisy with extra range noise.

A TEST fixture, not part of the system under test. The simulated LiDAR's own
noise is defined inside the upstream TurtleBot3 model, which this project does
not own and must not edit; relaying instead adds a known, seeded perturbation
on top of the nominal sensor without touching anyone else's package.

Only the TRACKER is pointed at the noisy topic (launch arg tracker_scan_topic).
AMCL and the obstacle costmap keep reading the clean /scan, so the experiment
isolates the tracker's tolerance to range noise rather than confounding it with
degraded localisation -- localisation is perturbed separately and deliberately
in its own condition.

The RNG is seeded per trial and the seed is logged, so a run is reproducible.
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNoise(Node):

    def __init__(self, sigma, seed, src, dst):
        super().__init__('scan_noise')
        self.sigma = float(sigma)
        self.rng = np.random.default_rng(int(seed))
        self.pub = self.create_publisher(LaserScan, dst, qos_profile_sensor_data)
        self.create_subscription(LaserScan, src, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'scan_noise: {src} -> {dst}, extra range sigma={self.sigma:.4f} m, seed={seed}')

    def cb(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        finite = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        if self.sigma > 0 and finite.any():
            r = r.copy()
            r[finite] += self.rng.normal(0.0, self.sigma, int(finite.sum()))
            # Keep the perturbed ranges physically valid; a range pushed outside
            # the sensor's declared band would be silently dropped downstream and
            # would act as a dropout rather than as noise.
            r[finite] = np.clip(r[finite], msg.range_min, msg.range_max)
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [float(x) for x in r]
        out.intensities = msg.intensities
        self.pub.publish(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sigma', type=float, default=0.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--src', default='/scan')
    ap.add_argument('--dst', default='/scan_noisy')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = ScanNoise(args.sigma, args.seed, args.src, args.dst)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
