#!/usr/bin/env python3

import numpy as np
import random
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class RoomMappingExplorer(Node):
    def __init__(self):
        super().__init__('room_mapping_explorer')

        # ROS interfaces
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_scan_callback, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.occupancy_grid_callback, 10)
        self.control_timer = self.create_timer(
            0.1, self.exploration_control_loop)

        # Latest sensor state
        self.latest_laser_scan = None
        self.latest_occupancy_grid = None
        self.obstacle_in_range = False
        self.nearest_obstacle_distance = float('inf')

        # Motion settings
        self.forward_speed = 0.22
        self.turn_speed = 0.4
        self.obstacle_distance_threshold = 0.5
        self.turn_direction = 1

        self.get_logger().info('Room Mapping Explorer node started.')
        self.get_logger().info('Robot will now explore and map the environment...')

    def laser_scan_callback(self, msg):
        """Update obstacle state from the latest LIDAR scan."""
        self.latest_laser_scan = msg

        # Replace infinite readings with the sensor's maximum range.
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

        self.nearest_obstacle_distance = np.min(ranges)
        self.obstacle_in_range = (
            self.nearest_obstacle_distance
            < self.obstacle_distance_threshold
        )

    def occupancy_grid_callback(self, msg):
        """Store the latest occupancy grid published by SLAM."""
        self.latest_occupancy_grid = msg

    def exploration_control_loop(self):
        """Publish velocity commands for exploration and obstacle avoidance."""
        if self.latest_laser_scan is None:
            return

        twist = Twist()

        if self.obstacle_in_range:
            # Choose the direction with more free space.
            self.get_logger().info(
                f'Obstacle detected at '
                f'{self.nearest_obstacle_distance:.2f}m - Avoiding...',
                throttle_duration_sec=2.0)

            twist.linear.x = 0.18

            ranges = np.array(self.latest_laser_scan.ranges)
            ranges[np.isinf(ranges)] = self.latest_laser_scan.range_max

            left_side = ranges[0:len(ranges)//3]
            right_side = ranges[2*len(ranges)//3:]

            left_distance = np.mean(left_side)
            right_distance = np.mean(right_side)

            if left_distance > right_distance:
                twist.angular.z = self.turn_speed
                self.get_logger().info('Rotating LEFT', throttle_duration_sec=2.0)
            else:
                twist.angular.z = -self.turn_speed
                self.get_logger().info(
                    'Rotating RIGHT', throttle_duration_sec=2.0)

        else:
            self.get_logger().info(
                f'Exploring... Nearest obstacle: '
                f'{self.nearest_obstacle_distance:.2f}m',
                throttle_duration_sec=3.0)

            twist.linear.x = self.forward_speed

            if random.random() < 0.05:  # 5% chance to change direction
                self.turn_direction *= -1

            twist.angular.z = self.turn_direction * 0.1 * random.random()

        self.velocity_publisher.publish(twist)

    def shutdown(self):
        """Stop the robot before shutting down."""
        self.get_logger().info('Shutting down - Stopping robot...')
        twist = Twist()
        self.velocity_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RoomMappingExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
