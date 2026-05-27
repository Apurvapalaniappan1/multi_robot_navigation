#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


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
        self.scans = {}
        self.avoidance_direction = {}

        self.k_formation = 0.8
        self.k_goal = 0.5
        self.k_angular = 1.5

        self.max_linear = 0.10
        self.max_angular = 0.6

        self.goal_tolerance = 0.20
        self.goal_reached_tolerance = 0.45
        self.max_neighbor_distance_seen = 0.0
        self.phase = 'forming'

        self.tb1_recovery_point = (-1.45, -1.35)
        self.tb1_recovery_done = False

        # Known obstacles from model.sdf.
        # The two_two box and one-one cylinder is intentionally excluded because it is treated as unknown/LiDAR-only.
        self.known_obstacles = [
             (-1.1, 1.1),   # one_three
             (1.1, -1.1),   # three_one
             (1.1, 1.1),    # three_three
        ]

        self.obstacle_radius = 0.15
        self.robot_radius = 0.18
        self.safe_distance = 0.55
        self.cbf_gain = 1.5

        # Connectivity maintenance parameters
        self.connectivity_soft_range = 1.10
        self.connectivity_critical_range = 1.25
        self.k_connectivity = 1.2
        self.max_connectivity_velocity = 0.12

        # Diamond path waypoints for the anchor/reference robot
        self.waypoints = [
        (1.6, 0.8),     # C - original TB3_3 position
        (-0.5, 2.0),    # D - old TB3_4 position
        (-2.0, -0.5),   # A - original TB3_1 position
        ]

        self.current_waypoint_index = 0

        self.pause_counter = 0
        self.pause_steps = 30


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
    
    def apply_known_obstacle_avoidance(self, robot, vx, vy):
        """
        CBF-inspired safety filter for known circular obstacles.
        It modifies the nominal velocity if the robot is too close
        to a known obstacle.
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

                safe_vx += repulse_x
                safe_vy += repulse_y

        return safe_vx, safe_vy
    
    def apply_robot_robot_avoidance(self, robot, vx, vy):
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

        return safe_vx, safe_vy
    
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
                  
               else :
                   self.get_logger().info(
                       f'{robot}: Connectivity maintenance active with {neighbor}, distance={distance:.2f} m'
                  )

               safe_vx += strength * (dx / distance)
               safe_vy += strength * (dy / distance)

               

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
        scan = self.scans.get(robot)

        if scan is None:
           return cmd
        
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

        lidar_safe_distance = 0.35
        critical_distance = 0.20

        if front_min < lidar_safe_distance:
           if front_min < critical_distance:
              cmd.linear.x = 0.0
           else:
              cmd.linear.x = min(cmd.linear.x, 0.05)

           if self.avoidance_direction[robot] == 0:
               if left_min > right_min:
                  self.avoidance_direction[robot] = 1
               else:
                  self.avoidance_direction[robot] = -1

           if self.avoidance_direction[robot] == 1:
              cmd.angular.z = 0.55
           else:
              cmd.angular.z = -0.55

           self.get_logger().info(
                f'{robot}: LiDAR unknown obstacle detected, avoiding around it.'
            )
        
        else:
          self.avoidance_direction[robot] = 0

        return cmd
    
    def formation_error_ok(self):
        max_error = 0.0

        for (robot, neighbor), (desired_dx, desired_dy) in self.desired_displacements.items():
            if robot not in self.poses or neighbor not in self.poses:
               continue

            xi, yi, _ = self.poses[robot]
            xj, yj, _ = self.poses[neighbor]

            actual_dx = xj - xi
            actual_dy = yj - yi

            error = math.sqrt(
                (actual_dx - desired_dx) ** 2 +
                (actual_dy - desired_dy) ** 2
            )

            max_error = max(max_error, error)

        return max_error < 0.55
    
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
                    cmd = Twist()
                    self.cmd_pubs[robot].publish(cmd)
                    continue

                if robot == 'TB3_1' and not self.tb1_recovery_done:
                   x, y, _ = self.poses[robot]
                   rx, ry = self.tb1_recovery_point

                   distance_to_recovery = math.sqrt((rx - x) ** 2 + (ry - y) ** 2)

                   if distance_to_recovery > 0.45:
                      ux = self.k_goal * (rx - x)
                      uy = self.k_goal * (ry - y)

                      self.get_logger().info(
                          f'TB3_1: recovery waypoint active, distance={distance_to_recovery:.2f} m'
                      )

                      ux, uy = self.apply_robot_robot_avoidance(robot, ux, uy)

                      cmd, reached = self.vector_to_cmd(robot, ux, uy)
                      #cmd = self.apply_lidar_avoidance(robot, cmd)
                      # Do not use LiDAR during initial recovery/forming.
                      # This waypoint exists only to route TB3_1 around the new obstacle.

                      all_reached = False
                      self.cmd_pubs[robot].publish(cmd)
                      continue

                   else:
                      self.tb1_recovery_done = True
                      self.get_logger().info(
                          'TB3_1: recovery waypoint reached. Returning to formation control.'
                      )

                ux, uy = self.formation_vector(robot)
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

        elif self.phase == 'moving':

            self.log_neighbor_distances()

            goal_x, goal_y = self.waypoints[self.current_waypoint_index]
            anchor_x, anchor_y, _ = self.poses[self.anchor]

            for robot in self.robots:
                if robot == self.anchor:
                   vx = self.k_goal * (goal_x - anchor_x)
                   vy = self.k_goal * (goal_y - anchor_y)

                   vx, vy = self.apply_known_obstacle_avoidance(robot, vx, vy)
                   vx, vy = self.apply_robot_robot_avoidance(robot, vx, vy)
                   
                else:
                   vx, vy = self.formation_vector(robot)
                
                   vx, vy = self.apply_known_obstacle_avoidance(robot, vx, vy)
                   vx, vy = self.apply_robot_robot_avoidance(robot, vx, vy)
                   vx, vy = self.apply_connectivity_maintenance(robot, vx, vy)

                cmd, _ = self.vector_to_cmd(robot, vx, vy)

                cmd = self.apply_lidar_avoidance(robot, cmd)

                self.cmd_pubs[robot].publish(cmd)


            distance_to_goal = math.sqrt(
                (goal_x - anchor_x) ** 2 + (goal_y - anchor_y) ** 2

            )
            
            self.get_logger().info(
                 f'Distance to goal: {distance_to_goal}'
            )

            if distance_to_goal < self.goal_reached_tolerance:
                self.get_logger().info(
                   f'Reached waypoint {self.current_waypoint_index + 1}.'
                )

                self.current_waypoint_index += 1

                if self.current_waypoint_index >= len(self.waypoints):
                   self.phase = 'final_reforming'
                   self.get_logger().info('Final waypoint reached. Reforming triangle.')
                   return

        elif self.phase == 'final_reforming':
            if self.formation_error_ok():
                self.get_logger().info('Triangle reformed. Mission complete.')
                stop_cmd = Twist()

                for robot in self.robots:
                   self.cmd_pubs[robot].publish(stop_cmd)

                self.phase = 'done'
                return

            for robot in self.robots:
                if robot == self.anchor:
                   cmd = Twist()
                   self.cmd_pubs[robot].publish(cmd)
                   continue

                vx, vy = self.formation_vector(robot)

                vx, vy = self.apply_known_obstacle_avoidance(robot, vx, vy)
                vx, vy = self.apply_robot_robot_avoidance(robot, vx, vy)
                vx, vy = self.apply_connectivity_maintenance(robot, vx, vy)

                cmd, _ = self.vector_to_cmd(robot, vx, vy)
                cmd = self.apply_lidar_avoidance(robot, cmd)

                self.cmd_pubs[robot].publish(cmd)

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
