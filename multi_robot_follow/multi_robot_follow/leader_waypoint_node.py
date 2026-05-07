#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class LeaderWaypointNode(Node):
    def __init__(self):
        super().__init__('leader_waypoint_node')

        self.x = None
        self.y = None
        self.yaw = 0.0

        # Adjust these after testing
        self.waypoints = [
            (-3.26, 1.76),   # start
            (-3.20, 0.70),   # move down inside left side
            (-3.20, -1.40),  # lower corridor
            (-1.40, -1.40),  # move right
            (0.60, -1.40),   # pass inner wall
            (0.80, 0.20),    # move upward around obstacle
            (2.40, 0.20),    # toward exit side
        ]
        self.current_wp = 0

        self.linear_gain = 0.45
        self.angular_gain = 1.6
        self.max_linear = 0.18
        self.max_angular = 0.9
        self.goal_tolerance = 0.25

        self.create_subscription(Odometry, '/TB3_1/odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/TB3_1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous leader waypoint node started.')

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def control_loop(self):
        if self.x is None or self.y is None:
            return

        cmd = Twist()

        if self.current_wp >= len(self.waypoints):
            self.cmd_pub.publish(cmd)
            return

        goal_x, goal_y = self.waypoints[self.current_wp]

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_wp + 1}')
            self.current_wp += 1
            self.cmd_pub.publish(cmd)
            return

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.yaw)

        # Turn first if facing wrong direction
        if abs(angle_error) > 0.35:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -self.max_angular,
                min(self.angular_gain * angle_error, self.max_angular)
            )
        else:
            cmd.linear.x = min(self.linear_gain * distance, self.max_linear)
            cmd.angular.z = max(
                -self.max_angular,
                min(self.angular_gain * angle_error, self.max_angular)
            )

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
    node = LeaderWaypointNode()

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