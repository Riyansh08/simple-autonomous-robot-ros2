import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class MovementController(Node):
    def __init__(self):
        super().__init__('movement_controller')
        self.get_logger().info("Movement Controller Initialized")

        # Create subscriber to LIDAR scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Publisher for robot velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Parameters for movement and control
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('safety_distance', 0.8)
        self.declare_parameter('turn_threshold', 0.5)
        self.declare_parameter('avoidance_gain', 1.5)

        # State variables
        self.current_twist = Twist()
        self.safe_to_move_forward = True
        self.obstacle_direction = 0.0

    def scan_callback(self, msg: LaserScan):
        """
        Callback for LIDAR scan data. This function processes the scan
        to determine obstacle proximity and adjusts the robot's movement accordingly.
        """
        # Process scan ranges and determine the minimum distance
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            self.get_logger().warn("No valid LIDAR data received.")
            return
        
        # Obstacle avoidance logic
        min_distance = np.min(valid_ranges)
        min_angle_index = np.argmin(valid_ranges)
        angle = msg.angle_min + min_angle_index * msg.angle_increment
        
        # Log current status
        self.get_logger().info(f'Min Distance: {min_distance:.2f}m at {math.degrees(angle):.2f}°')

        # Obstacle detection threshold
        safety_distance = self.get_parameter('safety_distance').value
        turn_threshold = self.get_parameter('turn_threshold').value
        
        if min_distance < safety_distance:
            self.get_logger().warn("Obstacle detected! Initiating avoidance maneuver.")
            self.safe_to_move_forward = False
            self.obstacle_direction = angle
            self.avoid_obstacle(angle, min_distance, turn_threshold)
        else:
            self.safe_to_move_forward = True
            self.move_forward()

    def avoid_obstacle(self, angle, distance, turn_threshold):
        """
        Implements obstacle avoidance using proportional control to turn the robot.
        """
        avoidance_gain = self.get_parameter('avoidance_gain').value
        angular_speed = self.get_parameter('angular_speed').value

        # Apply proportional control to steer around the obstacle
        turn_angle = -math.copysign(1, angle) * angular_speed * avoidance_gain
        
        if abs(angle) < turn_threshold:
            self.get_logger().info("Turning to avoid head-on collision.")
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = turn_angle
        else:
            self.get_logger().info("Obstacle at angle, proceeding with cautious turn.")
            self.current_twist.linear.x = 0.1
            self.current_twist.angular.z = turn_angle

        # Publish the movement command
        self.publisher_.publish(self.current_twist)

    def move_forward(self):
        """
        Moves the robot forward in a straight line if no obstacles are detected.
        """
        linear_speed = self.get_parameter('linear_speed').value
        self.current_twist.linear.x = linear_speed
        self.current_twist.angular.z = 0.0
        
        self.publisher_.publish(self.current_twist)
        self.get_logger().info("Moving forward.")

    def stop(self):
        """
        Stops the robot.
        """
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = 0.0
        self.publisher_.publish(self.current_twist)
        self.get_logger().info("Robot stopped.")

    def smooth_turn(self, direction='left', intensity=1.0):
        """
        Turns the robot smoothly based on the specified direction.
        """
        angular_speed = self.get_parameter('angular_speed').value
        turn_rate = angular_speed * intensity
        
        if direction == 'left':
            self.current_twist.angular.z = turn_rate
        else:
            self.current_twist.angular.z = -turn_rate
        
        self.publisher_.publish(self.current_twist)
        self.get_logger().info(f"Smooth turn to the {direction}.")

    def gradual_braking(self):
        """
        Gradually slows down the robot if an obstacle is detected.
        """
        while self.current_twist.linear.x > 0:
            self.current_twist.linear.x -= 0.02
            self.publisher_.publish(self.current_twist)
            self.get_logger().info("Gradual braking in progress.")
            self.sleep(0.1)
        self.stop()

    def stop_and_reverse(self):
        """
        Stops and reverses the robot slightly when an obstacle is detected.
        """
        self.stop()
        self.get_logger().info("Reversing to avoid obstacle.")
        self.current_twist.linear.x = -0.1
        self.publisher_.publish(self.current_twist)
        self.sleep(1.0)
        self.stop()

    def sleep(self, duration):
        """
        Helper function to simulate delay.
        """
        rclpy.spin_once(self, timeout_sec=duration)


def main(args=None):
    rclpy.init(args=args)
    node = MovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
