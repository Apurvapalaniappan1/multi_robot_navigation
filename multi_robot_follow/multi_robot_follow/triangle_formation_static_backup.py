#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TriangleFormation(Node):
    def __init__(self):
        super().__init__('triangle_formation')

        self.robots = ['TB3_1', 'TB3_2', 'TB3_3']

        # TB3_2 = anchor robot on right side
        self.anchor = 'TB3_2'

        # Desired triangle positions relative to anchor robot
        # TB3_2 stays at anchor point
        # TB3_1 goes lower-left of anchor
        # TB3_3 goes upper-left of anchor
        self.offsets = {
            'TB3_1': (-0.65, -0.45),  # lower-left of anchor
            'TB3_2': (0.0, 0.0),    # anchor/right-side robot
            'TB3_3': (-0.65, 0.45),   # upper-left of anchor
        }

        self.poses = {}

        self.k_linear = 0.5
        self.k_angular = 1.5
        self.max_linear = 0.08
        self.max_angular = 0.6
        self.goal_tolerance = 0.12
        self.formation_reached = False

        self.cmd_pubs = {}

        for robot in self.robots:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, robot=robot: self.odom_callback(msg, robot),
                10
            )

            self.cmd_pubs[robot] = self.create_publisher(
                Twist,
                f'/{robot}/cmd_vel',
                10
            )

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg, robot):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.poses[robot] = (x, y, yaw)

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

    def control_loop(self):

        if self.formation_reached:
           return

        if self.anchor not in self.poses:
           return

        anchor_x, anchor_y, _ = self.poses[self.anchor]

        all_reached = True

        for robot in self.robots:

            if robot not in self.poses:
               continue

            x, y, yaw = self.poses[robot]

            offset_x, offset_y = self.offsets[robot]

            target_x = anchor_x + offset_x
            target_y = anchor_y + offset_y

            dx = target_x - x
            dy = target_y - y

            distance = math.sqrt(dx * dx + dy * dy)

            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - yaw)

            cmd = Twist()

            if distance > self.goal_tolerance:

               all_reached = False

               cmd.linear.x = min(
                   self.k_linear * distance,
                   self.max_linear
                )

               cmd.angular.z = max(
                min(self.k_angular * angle_error,
                    self.max_angular),
                -self.max_angular
                )

            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            if robot == self.anchor:
               cmd.linear.x = 0.0
               cmd.angular.z = 0.0

            self.cmd_pubs[robot].publish(cmd)

        if all_reached:

           self.get_logger().info(
               'Triangle formation reached. Stopping robots.'
           )

           stop_cmd = Twist()

           for robot in self.robots:
               self.cmd_pubs[robot].publish(stop_cmd)

           self.formation_reached = True


def main(args=None):
    rclpy.init(args=args)
    node = TriangleFormation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
