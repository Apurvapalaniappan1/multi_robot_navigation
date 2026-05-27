#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DisplacementFormation(Node):
    def __init__(self):
        super().__init__('displacement_formation')

        self.robots = ['TB3_1', 'TB3_2', 'TB3_3']
        self.anchor = 'TB3_2'

        # Desired displacement q_j - q_i for each robot-neighbor pair
        self.desired_displacements = {
            ('TB3_1', 'TB3_2'): (0.65, 0.45),
            ('TB3_2', 'TB3_1'): (-0.65, -0.45),

            ('TB3_3', 'TB3_2'): (0.65, -0.45),
            ('TB3_2', 'TB3_3'): (-0.65, 0.45),
        }

        self.neighbors = {
            'TB3_1': ['TB3_2'],
            'TB3_2': ['TB3_1', 'TB3_3'],
            'TB3_3': ['TB3_2'],
        }

        self.poses = {}

        self.k_formation = 1.1
        self.k_goal = 0.7
        self.k_angular = 2.0

        self.max_linear = 0.16
        self.max_angular = 0.9

        self.goal_tolerance = 0.20
        self.goal_reached_tolerance = 0.45
        self.phase = 'forming'

        # Goal for the anchor/reference robot
        self.formation_goal = (-2.0, -0.5)

        self.pause_counter = 0
        self.pause_steps = 30

        self.tb3_1_clockwise_counter = 0
        self.tb3_1_clockwise_steps = 20

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

    def vector_to_cmd(self, robot, vx, vy):
        x, y, yaw = self.poses[robot]

        distance = math.sqrt(vx * vx + vy * vy)
        target_angle = math.atan2(vy, vx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        if distance < self.goal_tolerance:
            return cmd, True

        angle_threshold = 0.35

        if abs(angle_error) > angle_threshold:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                min(self.k_angular * angle_error, self.max_angular),
                -self.max_angular
            )
        else:
            cmd.linear.x = min(distance, self.max_linear)
            cmd.angular.z = max(
                min(self.k_angular * angle_error, self.max_angular),
                -self.max_angular
            )

        return cmd, False

    def formation_vector(self, robot):
        xi, yi, _ = self.poses[robot]

        ux = 0.0
        uy = 0.0

        for neighbor in self.neighbors[robot]:
            if neighbor not in self.poses:
                continue

            xj, yj, _ = self.poses[neighbor]

            desired_dx, desired_dy = self.desired_displacements[(robot, neighbor)]

            actual_dx = xj - xi
            actual_dy = yj - yi

            error_x = actual_dx - desired_dx
            error_y = actual_dy - desired_dy

            ux += self.k_formation * error_x
            uy += self.k_formation * error_y

        return ux, uy

    def control_loop(self):
        if not all(robot in self.poses for robot in self.robots):
            return

        if self.phase == 'forming':
            all_reached = True

            for robot in self.robots:
                if robot == self.anchor:
                    cmd = Twist()
                    self.cmd_pubs[robot].publish(cmd)
                    continue

                ux, uy = self.formation_vector(robot)
                formation_error = math.sqrt(ux**2 + uy**2)

                self.get_logger().info(
                    f'{robot}: formation error = {formation_error:.2f}'
                )
                cmd, reached = self.vector_to_cmd(robot, ux, uy)

                if not reached:
                    all_reached = False

                self.cmd_pubs[robot].publish(cmd)

            if all_reached:
                self.get_logger().info(
                    'Displacement-based triangle formation reached. Pausing.'
                )
                self.phase = 'pause'
                self.pause_counter = 0

        elif self.phase == 'pause':
            stop_cmd = Twist()

            for robot in self.robots:
                self.cmd_pubs[robot].publish(stop_cmd)

            self.pause_counter += 1

            if self.pause_counter >= self.pause_steps:
                self.get_logger().info(
                    'Pause complete. Moving formation.'
                )
                self.phase = 'moving'
                self.tb3_1_clockwise_counter = 0

        elif self.phase == 'moving':

            goal_x, goal_y = self.formation_goal
            anchor_x, anchor_y, _ = self.poses[self.anchor]

            for robot in self.robots:
                if robot == self.anchor:
                   vx = self.k_goal * (goal_x - anchor_x)
                   vy = self.k_goal * (goal_y - anchor_y)
                else:
                   vx, vy = self.formation_vector(robot)

                if robot == 'TB3_1' and self.tb3_1_clockwise_counter < self.tb3_1_clockwise_steps:
                   cmd = Twist()
                   cmd.linear.x = 0.0
                   cmd.angular.z = -0.4
                else:
                   cmd, _ = self.vector_to_cmd(robot, vx, vy)

                self.cmd_pubs[robot].publish(cmd)

            self.tb3_1_clockwise_counter += 1

            distance_to_goal = math.sqrt(
                (goal_x - anchor_x) ** 2 + (goal_y - anchor_y) ** 2

            )
            
            self.get_logger().info(
                 f'Distance to goal: {distance_to_goal}'
            )

            if distance_to_goal < self.goal_reached_tolerance:
                self.get_logger().info(
                   'Displacement-based moving formation reached goal. Stopping.'
                )
                
                stop_cmd = Twist()

                for robot in self.robots:
                    self.cmd_pubs[robot].publish(stop_cmd)

                self.phase = 'done'

        elif self.phase == 'done':
            return


def main(args=None):
    rclpy.init(args=args)
    node = DisplacementFormation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
