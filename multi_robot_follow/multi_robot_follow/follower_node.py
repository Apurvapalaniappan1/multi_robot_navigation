#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        self.leader = None
        self.follower = None
        self.follower_yaw = 0.0

        self.safe_distance = 0.7
        self.linear_gain = 0.5
        self.angular_gain = 1.5
        self.max_linear = 0.22
        self.max_angular = 1.0

        # Obstacle avoidance variables
        self.front_obstacle_distance = float('inf')
        self.obstacle_threshold = 0.45

        self.create_subscription(Odometry, '/TB3_1/odom', self.leader_cb, 10)
        self.create_subscription(Odometry, '/TB3_2/odom', self.follower_cb, 10)

        # LiDAR scan from follower robot
        self.create_subscription(LaserScan, '/TB3_2/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/TB3_2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Follower node started: TB3_2 follows TB3_1 and avoids obstacles')

    def leader_cb(self, msg):
        self.leader = msg.pose.pose.position

    def follower_cb(self, msg):
        self.follower = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.follower_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        # Front region of LiDAR: around 0 degrees
        front_ranges = ranges[0:20] + ranges[-20:]

        valid_ranges = [
            r for r in front_ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        if valid_ranges:
            self.front_obstacle_distance = min(valid_ranges)
        else:
            self.front_obstacle_distance = float('inf')

    def control_loop(self):
        if self.leader is None or self.follower is None:
            return

        dx = self.leader.x - self.follower.x
        dy = self.leader.y - self.follower.y
        distance = math.sqrt(dx * dx + dy * dy)

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.follower_yaw)

        cmd = Twist()

        # Priority 1: avoid obstacle in front
        if self.front_obstacle_distance < self.obstacle_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.cmd_pub.publish(cmd)
            return

        # Priority 2: follow leader
        if distance > self.safe_distance:
            cmd.linear.x = min(
                self.linear_gain * (distance - self.safe_distance),
                self.max_linear
            )
            cmd.angular.z = max(
                -self.max_angular,
                min(self.angular_gain * angle_error, self.max_angular)
            )
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_cmd = Twist()
    node.cmd_pub.publish(stop_cmd)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()