import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import numpy as np
import random
import math


class LidarSensorNode(Node):
    def __init__(self):
        super().__init__('lidar_sensor')
        self.get_logger().info("Lidar Sensor Node Initialized")
        
        # Publisher for LIDAR scan data
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Timer to periodically publish scan data
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        # Dynamic Parameters
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('angle_min', -1.57)  # -90 degrees
        self.declare_parameter('angle_max', 1.57)   # 90 degrees
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('noise_level', 0.05)

        # Obstacle Simulation
        self.obstacles = self.generate_random_obstacles()

        # Service to trigger sensor recalibration
        self.recalibrate_service = self.create_service(Empty, 'recalibrate_lidar', self.recalibrate_callback)

    def publish_scan(self):
        scan = LaserScan()

        # Time stamping
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_frame'
        
        # Fetch dynamic parameters
        angle_min = self.get_parameter('angle_min').value
        angle_max = self.get_parameter('angle_max').value
        angle_increment = self.get_parameter('angle_increment').value
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value
        noise_level = self.get_parameter('noise_level').value

        # Calculate number of data points
        num_readings = int((angle_max - angle_min) / angle_increment)

        # Generate LIDAR scan data
        ranges = []
        intensities = []

        for i in range(num_readings):
            angle = angle_min + i * angle_increment
            distance = self.simulate_lidar_beam(angle, range_min, range_max)
            noisy_distance = distance + random.uniform(-noise_level, noise_level)

            # Cap at max range
            if noisy_distance > range_max:
                noisy_distance = float('inf')

            ranges.append(noisy_distance)
            intensities.append(random.uniform(0.1, 1.0))  # Simulated intensity

        # Populate LaserScan message
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges
        scan.intensities = intensities
        
        # Publish scan data
        self.publisher_.publish(scan)
        self.get_logger().info(f'Published LIDAR Scan with {len(ranges)} points')

    def simulate_lidar_beam(self, angle, range_min, range_max):
        """
        Simulate the distance measured by a LIDAR beam at a specific angle.
        """
        beam_distance = range_max

        # Check for obstacle collision along the beam path
        for obs in self.obstacles:
            obs_distance = self.calculate_distance_to_obstacle(obs, angle)
            if range_min < obs_distance < beam_distance:
                beam_distance = obs_distance
        
        return beam_distance

    def calculate_distance_to_obstacle(self, obstacle, angle):
        """
        Calculate the distance from the LIDAR to a specific obstacle at a given angle.
        """
        ox, oy, radius = obstacle
        lidar_x, lidar_y = 0.0, 0.0  # LIDAR is at the origin

        beam_end_x = lidar_x + math.cos(angle) * 20
        beam_end_y = lidar_y + math.sin(angle) * 20

        dx = ox - lidar_x
        dy = oy - lidar_y
        projection = dx * math.cos(angle) + dy * math.sin(angle)

        if projection > 0:
            closest_distance = math.sqrt(dx**2 + dy**2) - radius
            return closest_distance
        
        return float('inf')

    def generate_random_obstacles(self):
        """
        Generate random obstacles to simulate a dynamic environment.
        """
        obstacles = []
        for _ in range(5):
            ox = random.uniform(-5.0, 5.0)
            oy = random.uniform(-5.0, 5.0)
            radius = random.uniform(0.2, 1.0)
            obstacles.append((ox, oy, radius))
        
        self.get_logger().info(f"Generated Obstacles: {obstacles}")
        return obstacles

    def recalibrate_callback(self, request, response):
        """
        Recalibrate LIDAR by regenerating obstacles.
        """
        self.obstacles = self.generate_random_obstacles()
        self.get_logger().info("LIDAR recalibrated with new obstacles.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
