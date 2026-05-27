#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CollisionAvoidanceOnly(Node):
    def __init__(self):
        super().__init__('collision_avoidance_only')

        self.robots = ['TB3_1', 'TB3_2', 'TB3_3']

        self.poses = {}
        self.scans = {}
        self.avoidance_direction = {}

        self.k_goal = 0.5
        self.k_angular = 1.5

        self.max_linear = 0.10
        self.max_angular = 0.6

        self.goal_tolerance = 0.20
        self.phase = 'moving'

        # Known obstacles handled by potential-field repulsion.
        # one_one and two_two boxes are NOT included here,
        # so they are treated as unknown LiDAR-only obstacles.
        self.known_obstacles = [
            (-1.1, 1.1),   # one_three
            (1.1, -1.1),   # three_one
            (1.1, 1.1),    # three_three
        ]

        self.obstacle_radius = 0.15
        self.robot_radius = 0.18
        self.safe_distance = 0.55
        self.cbf_gain = 1.5

        # Independent waypoint routes for each robot.
        self.waypoints = {
            'TB3_1': [
                (-1.75, -0.70),    # helper waypoint
                (0.5, -2.0),     # A -> B
            ],
            'TB3_2': [
                (1.6, 0.8),      # B -> C
                (-0.5, 2.0),     # C -> D
            ],
            'TB3_3': [
                (0.55, 1.35),      # helper waypoint
                (-0.5, 2.0),     # C -> D
                (-2.0, -0.5),    # D -> A
            ],
        }

        self.current_waypoint_index = {
            'TB3_1': 0,
            'TB3_2': 0,
            'TB3_3': 0,
        }

        self.cmd_pubs = {}

        for robot in self.robots:
            self.create_subscription(
                Odometry,
                f'/{robot}/odom',
                lambda msg, robot=robot: self.odom_callback(msg, robot),
                10
            )

            self.create_subscription(
                LaserScan,
                f'/{robot}/scan',
                lambda msg, robot=robot: self.scan_callback(msg, robot),
                10
            )

            self.scans[robot] = None

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

    def scan_callback(self, msg, robot):
        self.scans[robot] = msg

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

    def apply_known_obstacle_avoidance(self, robot, vx, vy):
        """
        Potential-field style repulsion for known obstacles.
        The nominal goal velocity is modified if the robot enters
        a safety region around a known obstacle.
        """
        x, y, _ = self.poses[robot]

        safe_vx = vx
        safe_vy = vy

        for obs_x, obs_y in self.known_obstacles:
            dx = x - obs_x
            dy = y - obs_y

            distance = math.sqrt(dx * dx + dy * dy)
            min_safe_distance = self.safe_distance + self.obstacle_radius

            if distance < min_safe_distance and distance > 0.001:
                repulsion_strength = self.cbf_gain * (min_safe_distance - distance)

                repulse_x = repulsion_strength * (dx / distance)
                repulse_y = repulsion_strength * (dy / distance)

                # Repulsive push away from obstacle
                safe_vx += repulse_x
                safe_vy += repulse_y

                # Tangential component helps robot move around obstacle instead of getting stuck
                tangent_strength = 0.15
                tangent_x = -dy / distance
                tangent_y = dx / distance

                safe_vx += tangent_strength * tangent_x
                safe_vy += tangent_strength * tangent_y

                self.get_logger().info(
                    f'{robot}: avoiding known obstacle at ({obs_x}, {obs_y})'
                )

        return safe_vx, safe_vy

    def apply_robot_robot_avoidance(self, robot, vx, vy):
        """
        Potential-field style repulsion between robots.
        Each robot treats the other robots as dynamic obstacles.
        """
        x, y, _ = self.poses[robot]

        safe_vx = vx
        safe_vy = vy

        min_safe_distance = 2.0 * self.robot_radius + self.safe_distance

        for other_robot in self.robots:
            if other_robot == robot:
                continue

            if other_robot not in self.poses:
                continue

            ox, oy, _ = self.poses[other_robot]

            dx = x - ox
            dy = y - oy

            distance = math.sqrt(dx * dx + dy * dy)

            if distance < min_safe_distance and distance > 0.001:
                repulsion_strength = self.cbf_gain * (min_safe_distance - distance)

                repulse_x = repulsion_strength * (dx / distance)
                repulse_y = repulsion_strength * (dy / distance)

                safe_vx += repulse_x
                safe_vy += repulse_y

                self.get_logger().info(
                    f'{robot}: avoiding nearby robot {other_robot}'
                )

        return safe_vx, safe_vy

    def get_min_distance(self, ranges):
        valid_ranges = [
            r for r in ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        if len(valid_ranges) == 0:
            return 999.0

        return min(valid_ranges)

    def apply_lidar_avoidance(self, robot, cmd):
        """
        Reactive LiDAR avoidance for unknown obstacles.
        This is used for boxes that are not listed in known_obstacles.
        """
        scan = self.scans.get(robot)

        if scan is None:
            return cmd

        # If close to a known obstacle, let known-obstacle avoidance handle it.
        if self.near_known_obstacle(robot):
            return cmd

        if robot not in self.avoidance_direction:
            self.avoidance_direction[robot] = 0

        ranges = scan.ranges

        front = ranges[0:20] + ranges[-20:]
        left = ranges[40:100]
        right = ranges[-100:-40]

        front_min = self.get_min_distance(front)
        left_min = self.get_min_distance(left)
        right_min = self.get_min_distance(right)

        lidar_safe_distance = 0.45
        critical_distance = 0.25

        if front_min < lidar_safe_distance:
            if front_min < critical_distance:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = 0.04

            if self.avoidance_direction[robot] == 0:
                if left_min > right_min:
                    self.avoidance_direction[robot] = 1
                else:
                    self.avoidance_direction[robot] = -1

            if self.avoidance_direction[robot] == 1:
                cmd.angular.z = 0.75
            else:
                cmd.angular.z = -0.75

            self.get_logger().info(
                f'{robot}: LiDAR unknown obstacle detected, avoiding around it.'
            )

        else:
            self.avoidance_direction[robot] = 0

        return cmd

    def near_known_obstacle(self, robot):
        x, y, _ = self.poses[robot]

        for obs_x, obs_y in self.known_obstacles:
            distance = math.sqrt(
                (x - obs_x) ** 2 +
                (y - obs_y) ** 2
            )

            if distance < 0.75:
                return True

        return False

    def control_loop(self):
        if not all(robot in self.poses for robot in self.robots):
           return

        if self.phase == 'moving':
           all_done = True

           for robot in self.robots:
               if self.current_waypoint_index[robot] >= len(self.waypoints[robot]):
                  stop_cmd = Twist()
                  self.cmd_pubs[robot].publish(stop_cmd)
                  continue

               all_done = False

               x, y, _ = self.poses[robot]
               waypoint_index = self.current_waypoint_index[robot]
               goal_x, goal_y = self.waypoints[robot][waypoint_index]

               distance_to_goal = math.sqrt(
                  (goal_x - x) ** 2 +
                  (goal_y - y) ** 2
               )

               self.get_logger().info(
                   f'{robot}: moving to waypoint {waypoint_index + 1}, distance = {distance_to_goal:.2f} m'
               )

               vx = self.k_goal * (goal_x - x)
               vy = self.k_goal * (goal_y - y)

               vx, vy = self.apply_known_obstacle_avoidance(robot, vx, vy)
               vx, vy = self.apply_robot_robot_avoidance(robot, vx, vy)

               cmd, reached = self.vector_to_cmd(robot, vx, vy)
               cmd = self.apply_lidar_avoidance(robot, cmd)

               if reached or distance_to_goal < self.goal_tolerance:
                  self.get_logger().info(
                      f'{robot}: reached waypoint {waypoint_index + 1}.'
                    )
                  self.current_waypoint_index[robot] += 1
                  cmd = Twist()

               self.cmd_pubs[robot].publish(cmd)

           if all_done:
              self.get_logger().info(
                  'Collision avoidance demo complete. All robots completed waypoint routes.'
                )

              stop_cmd = Twist()
              for robot in self.robots:
                  self.cmd_pubs[robot].publish(stop_cmd)

              self.phase = 'done'

        elif self.phase == 'done':
              return
        
def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceOnly()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()