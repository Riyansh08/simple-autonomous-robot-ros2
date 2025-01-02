import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import numpy as np
import math
import time


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.get_logger().info("Robot Controller Node Initialized")

        # LIDAR Subscriber
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)

        # Velocity Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # State Publisher (For logging and external monitoring)
        self.state_publisher = self.create_publisher(String, 'robot_state', 10)

        # Control Loop Timer
        self.control_loop_timer = self.create_timer(0.1, self.control_loop)

        # Declare dynamic parameters
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('obstacle_distance_threshold', 1.0)
        self.declare_parameter('free_path_distance', 2.0)

        # State Variables
        self.obstacle_detected = False
        self.lidar_ranges = []
        self.current_state = "IDLE"
        self.lock = threading.Lock()  # For thread-safe operations
        self.stop_command_received = False

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LIDAR data, updating obstacle detection state.
        """
        with self.lock:
            self.lidar_ranges = np.array(msg.ranges)
            if np.any(np.isfinite(self.lidar_ranges)):
                min_distance = np.min(self.lidar_ranges[np.isfinite(self.lidar_ranges)])
                self.obstacle_detected = min_distance < self.get_parameter('obstacle_distance_threshold').value

    def control_loop(self):
        """
        Main control loop to adjust robot behavior based on sensor data.
        """
        with self.lock:
            if self.stop_command_received:
                self.stop_robot()
                return

            if self.obstacle_detected:
                self.current_state = "AVOIDING_OBSTACLE"
                self.avoid_obstacle()
            else:
                self.current_state = "MOVING_FORWARD"
                self.move_forward()

            # Publish robot state for monitoring
            state_msg = String()
            state_msg.data = f"Current State: {self.current_state}"
            self.state_publisher.publish(state_msg)

    def move_forward(self):
        """
        Command to move the robot forward if no obstacles are detected.
        """
        twist = Twist()
        linear_speed = self.get_parameter('linear_speed').value
        twist.linear.x = linear_speed
        twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Moving forward.")

    def avoid_obstacle(self):
        """
        Implements obstacle avoidance using proportional control.
        """
        twist = Twist()
        angular_speed = self.get_parameter('angular_speed').value

        # Calculate the safest direction to turn
        if len(self.lidar_ranges) > 0:
            left_side = np.mean(self.lidar_ranges[:len(self.lidar_ranges) // 4])
            right_side = np.mean(self.lidar_ranges[-len(self.lidar_ranges) // 4:])

            if right_side > left_side:
                self.get_logger().info("Obstacle detected. Turning right.")
                twist.angular.z = -angular_speed
            else:
                self.get_logger().info("Obstacle detected. Turning left.")
                twist.angular.z = angular_speed

        twist.linear.x = 0.1  # Move slowly while turning
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        """
        Immediately stops the robot.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Emergency stop initiated.")
        self.current_state = "STOPPED"

    def stop_command_listener(self):
        """
        Simulates listening to external stop signals (for real systems, this could be a service).
        """
        while True:
            command = input("Type 'stop' to halt the robot: ").strip().lower()
            if command == 'stop':
                with self.lock:
                    self.stop_command_received = True
                self.get_logger().warn("STOP command received.")
                break

    def gradual_stop(self):
        """
        Gradually decelerates the robot instead of abrupt stops.
        """
        twist = Twist()
        while self.get_parameter('linear_speed').value > 0:
            current_speed = self.get_parameter('linear_speed').value
            current_speed -= 0.05
            twist.linear.x = max(0, current_speed)
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def emergency_reverse(self):
        """
        Reverses the robot slightly after detecting a close-range obstacle.
        """
        twist = Twist()
        twist.linear.x = -0.1
        self.cmd_vel_publisher.publish(twist)
        time.sleep(1.0)
        self.stop_robot()

    def run(self):
        """
        Starts the robot controller and enables external listener threads.
        """
        stop_listener_thread = threading.Thread(target=self.stop_command_listener)
        stop_listener_thread.start()


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    controller.run()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
