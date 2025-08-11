import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg  import Twist
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np

from .dynamixel_controller import Dynamixel


class GoatController(Node):
    # Constants
    DYNA_TO_AMP = 2.69e-3  # Converts Dynamixel units [int] to [A]
    DYNA_TO_REV_PER_MIN = 0.229  # Converts Dynamixel units [int] to [rev/min]
    WHEEL_RADIUS = 0.171  # Wheel radius [m]

    # Motor IDs and Directions
    ID_FRONT_LEFT = 11
    ID_FRONT_RIGHT = 14
    ID_BACK_LEFT = 12
    ID_BACK_RIGHT = 13

    DIR_FRONT_LEFT = -1
    DIR_FRONT_RIGHT = 1
    DIR_BACK_LEFT = -1
    DIR_BACK_RIGHT = 1

    def __init__(self):
        super().__init__("goat_controller")

        # Configuration flags
        self.check_error = False

        # State variables
        self.linear_velocity: np.ndarray = np.zeros(3)
        self.linear_velocity_smooth: np.ndarray = np.zeros(3)
        self.linear_acceleration: np.ndarray = np.zeros(3)
        self.angular_velocity: np.ndarray = np.zeros(3)
        self.angular_velocity_smooth: np.ndarray = np.zeros(3)
        self.angular_acceleration: np.ndarray = np.zeros(3)
        self.frame_points: np.ndarray = np.zeros(12 * 3)

        # Parameters
        self._declare_parameters()
        self._setup_communication()
        self._setup_publishers_subscribers()

        # Timer
        self.timer_period = 0.05  # 20Hz
        self.state_timer = self.create_timer(self.timer_period, self._state_callback)

    def _declare_parameters(self):
        """Declare and get all ROS parameters"""
        # Default control parameters
        self._frame_width = self.declare_parameter("frame_width", 0.36).get_parameter_value().double_value

        # Control gains
        self.linear_p = self.declare_parameter("linear_p", 0.0).get_parameter_value().double_value
        self.linear_d = self.declare_parameter("linear_d", 0.0).get_parameter_value().double_value
        self.linear_alpha = self.declare_parameter("linear_alpha", 0.9).get_parameter_value().double_value
        self.angular_p = self.declare_parameter("angular_p", 1.0).get_parameter_value().double_value
        self.angular_d = self.declare_parameter("angular_d", 0.1).get_parameter_value().double_value
        self.angular_alpha = self.declare_parameter("angular_alpha", 0.9).get_parameter_value().double_value

        # Topic names
        self.joystick_topic = self.declare_parameter("joystick_topic", "/joy").get_parameter_value().string_value
        self.commanded_velocity_topic = (
            self.declare_parameter("commanded_velocity_topic", "/commanded_velocity").get_parameter_value().string_value
        )
        self.measured_velocity_topic = (
            self.declare_parameter("measured_velocity_topic", "/measured_velocity").get_parameter_value().string_value
        )
        self.current_consumption_topic = (
            self.declare_parameter("current_consumption_topic", "/current_consumption").get_parameter_value().string_value
        )
        self.linear_velocity_topic = self.declare_parameter("linear_velocity_topic", "/linear_velocity").get_parameter_value().string_value
        self.angular_velocity_topic = (
            self.declare_parameter("angular_velocity_topic", "/angular_velocity").get_parameter_value().string_value
        )
        self.desired_twist_topic = self.declare_parameter("desired_twist", "/desired_twist").get_parameter_value().string_value
        self.estimated_width_topic = self.declare_parameter("estimated_width", "/estimated_width").get_parameter_value().string_value
        self.frame_points_topic = self.declare_parameter("frame_points_topic", "/frame_points").get_parameter_value().string_value

        # Scaling factors
        self.linear_scale = self.declare_parameter("linear_scale", 0.4).get_parameter_value().double_value
        self.angular_scale = self.declare_parameter("angular_scale", 1.0).get_parameter_value().double_value

    def _setup_communication(self):
        """Initialize communication with Dynamixel servos"""
        self.servo = Dynamixel(
            ID=[self.ID_FRONT_LEFT, self.ID_FRONT_RIGHT, self.ID_BACK_LEFT, self.ID_BACK_RIGHT],
            descriptive_device_name="DYNAMIXEL_GOAT",
            port_name="/dev/ttyUSB0",
            baudrate=1000000,
            series_name=["xm", "xm", "xm", "xm"],
        )
        self.servo.begin_communication()
        self.servo.disable_torque(False, ID="all")
        self.servo.set_current_limit(1000, ID="all")
        self.servo.set_operating_mode("velocity", ID="all")
        self.servo.enable_torque(False, ID="all")

    def _setup_publishers_subscribers(self):
        """Create all publishers and subscribers"""
        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self.joystick_callback, 10)
        self.commanded_velocity_publisher = self.create_publisher(Float32MultiArray, self.commanded_velocity_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32MultiArray, self.measured_velocity_topic, 10)
        self.current_consumption_publisher = self.create_publisher(Float32MultiArray, self.current_consumption_topic, 10)
        self.desired_twist_publisher = self.create_publisher(Twist, self.desired_twist_topic, 10)
        self.estimated_width_publisher = self.create_publisher(Float32, self.estimated_width_topic, 10)
        self.linear_velocity_subscription = self.create_subscription(
            Float32MultiArray, self.linear_velocity_topic, self.linear_velocity_callback, 10
        )
        self.angular_velocity_subscription = self.create_subscription(
            Float32MultiArray, self.angular_velocity_topic, self.angular_velocity_callback, 10
        )
        self.frme_points_subscription = self.create_subscription(
            Float32MultiArray, self.frame_points_topic, self.frame_points_callback, 10
        )

        self.get_logger().info(f"Subscribed to {self.joystick_topic}")

    def _compute_wheel_velocities(self, linear: float, angular: float) -> tuple[float, float]:
        """Convert linear [m/s] and angular velocity [rad/s] to left/right wheel velocities [rev/min]"""
        linear_rev_per_min = linear / (2.0 * np.pi * self.WHEEL_RADIUS) * 60
        angular_rev_per_min = angular * 0.5 * self._frame_width / (2.0 * np.pi * self.WHEEL_RADIUS) * 60
        left = linear_rev_per_min - angular_rev_per_min
        right = linear_rev_per_min + angular_rev_per_min
        return left, right

    def send_wheel_velocity(self, left: float, right: float):
        """Send velocity commands to Dynamixel motors"""
        left_wheel_dynamixel_velocity = int(left / self.DYNA_TO_REV_PER_MIN)
        right_wheel_dynamixel_velocity = int(right / self.DYNA_TO_REV_PER_MIN)

        vels = [
            self.DIR_FRONT_LEFT * left_wheel_dynamixel_velocity,
            self.DIR_FRONT_RIGHT * right_wheel_dynamixel_velocity,
            self.DIR_BACK_LEFT * left_wheel_dynamixel_velocity,
            self.DIR_BACK_RIGHT * right_wheel_dynamixel_velocity,
        ]
        self.servo.write_velocity(vels, "all")

    def publish_wheel_velocity(self, left: float, right: float):
        """Publish commanded wheel velocities"""
        commanded_velocity_msg = Float32MultiArray()
        commanded_velocity_msg.data = [left, right]
        self.commanded_velocity_publisher.publish(commanded_velocity_msg)

    def publish_desired_twist(self, twist_msg: Twist):
        """Publish desired base velocities"""
        self.desired_twist_publisher.publish(twist_msg)

    def joystick_callback(self, msg: Joy):
        """Handle joystick input and compute wheel velocities"""
        left_wheel_velocity = right_wheel_velocity = 0.0
        desired_linear = desired_angular = 0.0

        if abs(msg.axes[1]) > 0.1 or abs(msg.axes[0]) > 0.1:
            # Direct control mode
            desired_linear = self.linear_scale * msg.axes[1]  # [m/s]
            desired_angular = self.angular_scale * msg.axes[0]  # [m/s]
            left_wheel_velocity, right_wheel_velocity = self._compute_wheel_velocities(desired_linear, desired_angular)
        elif abs(msg.axes[4]) > 0.1 or abs(msg.axes[3]) > 0.1:
            # PID control mode
            desired_linear = self.linear_scale * msg.axes[4]
            desired_angular = self.angular_scale * msg.axes[3]

            linear_error = desired_linear - self.linear_velocity[0]
            angular_error = desired_angular - self.angular_velocity[0]

            linear = desired_linear + self.linear_p * linear_error - self.linear_d * self.linear_acceleration[0]
            angular = desired_angular + self.angular_p * angular_error - self.angular_d * self.angular_acceleration[0]

            left_wheel_velocity, right_wheel_velocity = self._compute_wheel_velocities(linear, angular)

        desired_twist = Twist()
        desired_twist.linear.x = desired_linear
        desired_twist.angular.z = desired_angular
        self.publish_desired_twist(desired_twist)
        self.send_wheel_velocity(left_wheel_velocity, right_wheel_velocity)
        self.publish_wheel_velocity(left_wheel_velocity, right_wheel_velocity)

    def linear_velocity_callback(self, msg: Float32MultiArray):
        """Update linear velocity with exponential smoothing"""
        self.linear_velocity = np.array(msg.data)
        old_smooth = self.linear_velocity_smooth
        self.linear_velocity_smooth = self.linear_velocity_smooth * self.linear_alpha + self.linear_velocity * (1.0 - self.linear_alpha)
        self.linear_acceleration = (self.linear_velocity_smooth - old_smooth) / self.timer_period

    def angular_velocity_callback(self, msg: Float32MultiArray):
        """Update angular velocity with exponential smoothing"""
        self.angular_velocity = np.array(msg.data)
        old_smooth = self.angular_velocity_smooth
        self.angular_velocity_smooth = self.angular_velocity_smooth * self.angular_alpha + self.angular_velocity * (
            1.0 - self.angular_alpha
        )
        self.angular_acceleration = (self.angular_velocity_smooth - old_smooth) / self.timer_period

    def frame_points_callback(self, msg: Float32MultiArray):
        """Update GOAT frame point vector and frame width for angular velocity calculations"""
        self.frame_points = np.array(msg.data).reshape(3, 12)
        avg_distance = np.mean(self.frame_points[[1, 3, 8, 9], :] - self.frame_points[[5, 7, 10, 11], :], axis=1)
        self._frame_width = np.linalg.norm(avg_distance)
        estimated_width = Float32()
        estimated_width.data = self._frame_width
        self.estimated_width_publisher.publish(estimated_width)

    def _state_callback(self):
        """Timer callback for reading and publishing motor states"""
        # Check for errors if enabled
        if self.check_error:
            errors = self.servo.get_errors(ID="all")
            for motor_id, error in errors:
                self.get_logger().info(f"Drive {motor_id} has error {error}")

        # Read and process wheel velocities
        wheel_velocity = self.servo.read_velocity("all")
        if not wheel_velocity:
            self.get_logger().warn("Failed to read wheel velocities!")
            return

        # Apply direction and scaling
        wheel_velocity = [
            wheel_velocity[0] * self.DIR_FRONT_LEFT * self.DYNA_TO_REV_PER_MIN,
            wheel_velocity[1] * self.DIR_FRONT_RIGHT * self.DYNA_TO_REV_PER_MIN,
            wheel_velocity[2] * self.DIR_BACK_LEFT * self.DYNA_TO_REV_PER_MIN,
            wheel_velocity[3] * self.DIR_BACK_RIGHT * self.DYNA_TO_REV_PER_MIN,
        ]

        # Publish measured velocity
        measured_velocity_msg = Float32MultiArray()
        measured_velocity_msg.data = wheel_velocity
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        # Read and publish current consumption
        wheel_current = self.servo.read_current("all")
        if wheel_current:
            wheel_current = [curr * self.DYNA_TO_AMP for curr in wheel_current]
            current_msg = Float32MultiArray()
            current_msg.data = wheel_current
            self.current_consumption_publisher.publish(current_msg)

    def __del__(self):
        """Cleanup on destruction"""
        if hasattr(self, "servo"):
            self.servo.disable_torque(ID="all")


def main(args=None):
    rclpy.init(args=args)
    controller = GoatController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
