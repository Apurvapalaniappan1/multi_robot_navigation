#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ConnectivityMaintenanceOnly(Node):
    def __init__(self):
        super().__init__('connectivity_maintenance_only')

        self.robots = ['TB3_1', 'TB3_2', 'TB3_3']
        self.anchor = 'TB3_2'

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
        self.cmd_pubs = {}

        self.k_formation = 1.4
        self.k_goal = 0.65
        self.k_angular = 1.5

        self.max_linear = 0.20
        self.max_angular = 0.7

        self.goal_tolerance = 0.35
        self.goal_reached_tolerance = 0.55

        self.connectivity_soft_range = 0.85
        self.connectivity_critical_range = 1.00
        self.k_connectivity = 2.0
        self.max_connectivity_velocity = 0.20

        self.demo_counter = 0
        self.boost_start_steps = 15
        self.boost_end_steps = 60

        self.phase = 'forming'
        self.pause_counter = 0
        self.pause_steps = 25

        # Far anchor goal to stretch the network and trigger connectivity maintenance
        self.anchor_goal = (-0.5, 2.0)

        self.max_neighbor_distance_seen = 0.0

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
        _, _, yaw = self.poses[robot]

        distance = math.sqrt(vx * vx + vy * vy)
        target_angle = math.atan2(vy, vx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        if distance < self.goal_tolerance:
            return cmd, True

        if abs(angle_error) > 0.35:
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(self.k_angular * angle_error, self.max_angular), -self.max_angular)
        else:
            cmd.linear.x = min(distance, self.max_linear)
            cmd.angular.z = max(min(self.k_angular * angle_error, self.max_angular), -self.max_angular)

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

    def apply_connectivity_maintenance(self, robot, vx, vy):
        x, y, _ = self.poses[robot]

        safe_vx = vx
        safe_vy = vy

        for neighbor in self.neighbors[robot]:
            if neighbor not in self.poses:
                continue

            nx, ny, _ = self.poses[neighbor]

            dx = nx - x
            dy = ny - y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < 0.001:
                continue

            if distance > self.connectivity_soft_range:
                stretch = distance - self.connectivity_soft_range
                strength = self.k_connectivity * stretch
                strength = min(strength, self.max_connectivity_velocity)

                if distance > self.connectivity_critical_range:
                    strength = self.max_connectivity_velocity
                    self.get_logger().info(
                        f'{robot}: CRITICAL connectivity recovery with {neighbor}, distance={distance:.2f} m'
                    )
                else:
                    self.get_logger().info(
                        f'{robot}: Connectivity maintenance active with {neighbor}, distance={distance:.2f} m'
                    )

                safe_vx += strength * (dx / distance)
                safe_vy += strength * (dy / distance)

        return safe_vx, safe_vy

    def log_neighbor_distances(self):
        for robot in self.robots:
            for neighbor in self.neighbors[robot]:
                if robot < neighbor:
                    xi, yi, _ = self.poses[robot]
                    xj, yj, _ = self.poses[neighbor]

                    distance = math.sqrt((xj - xi) ** 2 + (yj - yi) ** 2)

                    if distance > self.max_neighbor_distance_seen:
                        self.max_neighbor_distance_seen = distance
                        self.get_logger().info(
                            f'NEW_MAX_NEIGHBOR_DISTANCE {robot}-{neighbor}: {distance:.2f} m'
                        )

    def control_loop(self):
        if not all(robot in self.poses for robot in self.robots):
            return

        if self.phase == 'forming':
            all_reached = True

            for robot in self.robots:
                if robot == self.anchor:
                    self.cmd_pubs[robot].publish(Twist())
                    continue

                vx, vy = self.formation_vector(robot)
                formation_error = math.sqrt(vx * vx + vy * vy)

                #self.get_logger().info(
                   # f'{robot}: formation error = {formation_error:.2f}')

                cmd, reached = self.vector_to_cmd(robot, vx, vy)

                if not reached:
                    all_reached = False

                self.cmd_pubs[robot].publish(cmd)

            if all_reached:
                self.get_logger().info('Initial triangle formation reached. Pausing before connectivity demo.')
                self.phase = 'pause'
                self.pause_counter = 0

        elif self.phase == 'pause':
            for robot in self.robots:
                self.cmd_pubs[robot].publish(Twist())

            self.pause_counter += 1

            if self.pause_counter >= self.pause_steps:
                self.get_logger().info('Pause complete. Starting connectivity maintenance demo.')
                self.phase = 'connectivity_demo'

        elif self.phase == 'connectivity_demo':
            self.log_neighbor_distances()

            goal_x, goal_y = self.anchor_goal
            anchor_x, anchor_y, _ = self.poses[self.anchor]

            self.demo_counter +=1

            for robot in self.robots:
                if robot == self.anchor:
                   if self.boost_start_steps <= self.demo_counter <= self.boost_end_steps:
                      boosted_k_goal = 2.5
                      current_max_linear = 0.28

                      self.get_logger().info(
                          'TB3_2: intentional communication stretch test active.'
                      )
                   else:
                      boosted_k_goal = self.k_goal
                      current_max_linear = 0.16

                   vx = boosted_k_goal * (goal_x - anchor_x)
                   vy = boosted_k_goal * (goal_y - anchor_y)

                   old_max_linear = self.max_linear
                   self.max_linear = current_max_linear
                   cmd, _ = self.vector_to_cmd(robot, vx, vy)
                   self.max_linear = old_max_linear
                   
                else:
                    vx, vy = self.formation_vector(robot)
                    vx, vy = self.apply_connectivity_maintenance(robot, vx, vy)
                    cmd, _ = self.vector_to_cmd(robot, vx, vy)

                self.cmd_pubs[robot].publish(cmd)

            distance_to_goal = math.sqrt((goal_x - anchor_x) ** 2 + (goal_y - anchor_y) ** 2)

            self.get_logger().info(
                f'Anchor distance to goal: {distance_to_goal:.2f} m'
            )

            if distance_to_goal < self.goal_reached_tolerance:
               self.get_logger().info('Connectivity maintenance demo complete. Stopping robots.')

               for robot in self.robots:
                   self.cmd_pubs[robot].publish(Twist())

               self.phase = 'done'

        elif self.phase == 'done':
            return


def main(args=None):
    rclpy.init(args=args)
    node = ConnectivityMaintenanceOnly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()