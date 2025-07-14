import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

from .dynamixel_controller import Dynamixel


class GoatController(Node):
    def __init__(self):
        super().__init__("goat_controller")

        self.check_error = False

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

        self.linear_scale = self.declare_parameter("linear_scale", 0.5).get_parameter_value().double_value
        self.angular_scale = self.declare_parameter("angular_scale", 0.5).get_parameter_value().double_value

        self.joystick_subscription = self.create_subscription(Joy, self.joystick_topic, self.joystick_callback, 10)
        self.commanded_velocity_publisher = self.create_publisher(Float32MultiArray, self.commanded_velocity_topic, 10)
        self.measured_velocity_publisher = self.create_publisher(Float32MultiArray, self.measured_velocity_topic, 10)
        self.current_consumption_publisher = self.create_publisher(Float32MultiArray, self.current_consumption_topic, 10)

        self.get_logger().info(f"Subscribed to {self.joystick_topic}")

        timer_period = 0.05  # seconds -> 20Hz
        self.state_timer = self.create_timer(timer_period, self._state_callback)

        self.servo = Dynamixel(
            ID=[11, 12, 13, 14],
            descriptive_device_name="DYNAMIXEL_GOAT",
            port_name="/dev/ttyUSB0",
            baudrate=1000000,
            series_name=["xm", "xm", "xm", "xm"],
        )
        self.servo.begin_communication()
        self.servo.disable_torque(False, ID="all")  # Should be done before messing with gains and such
        self.servo.set_current_limit(1000, ID="all")
        self.servo.set_operating_mode("velocity", ID="all")
        self.servo.set_velocity_pid(100, 1920, 0, ID="all")
        self.servo.enable_torque(False, ID="all")

        self.ID_FRONT_LEFT = 11
        self.ID_BACK_LEFT = 12
        self.ID_FRONT_RIGHT = 14
        self.ID_BACK_RIGHT = 13

        self.DIR_FRONT_LEFT = -1
        self.DIR_BACK_LEFT = -1
        self.DIR_FRONT_RIGHT = 1
        self.DIR_BACK_RIGHT = 1

    def joystick_callback(self, msg: Joy):

        linear_velocity = 0.0
        angular_velocity = 0.0

        if abs(msg.axes[1]) > 0.1 or abs(msg.axes[0]) > 0.1:
            linear_velocity = self.linear_scale * msg.axes[1]
            angular_velocity = self.angular_scale * msg.axes[0]

        left_wheel_velocity = linear_velocity - angular_velocity
        right_wheel_velocity = linear_velocity + angular_velocity

        left_wheel_dynamixel_velocity = int(left_wheel_velocity * 310)
        right_wheel_dynamixel_velocity = int(right_wheel_velocity * 310)
        ids = [self.ID_FRONT_LEFT, self.ID_FRONT_RIGHT, self.ID_BACK_LEFT, self.ID_BACK_RIGHT]
        vels = [
            self.DIR_FRONT_LEFT * left_wheel_dynamixel_velocity,
            self.DIR_FRONT_RIGHT * right_wheel_dynamixel_velocity,
            self.DIR_BACK_LEFT * left_wheel_dynamixel_velocity,
            self.DIR_BACK_RIGHT * right_wheel_dynamixel_velocity,
        ]

        self.servo.write_velocity(vels, ids)

        left_wheel_velocity = left_wheel_dynamixel_velocity * 0.229
        right_wheel_velocity = right_wheel_dynamixel_velocity * 0.229

        # Publish commanded velocity
        commanded_velocity_msg = Float32MultiArray()
        commanded_velocity_msg.data = [left_wheel_velocity, right_wheel_velocity]
        self.commanded_velocity_publisher.publish(commanded_velocity_msg)

    def _state_callback(self):
        # Read errors
        if self.check_error:
            errors = self.servo.get_errors(ID="all")
            for error in errors:
                self.get_logger().info(f"Drive {error[0]} has error {error[1]}")

        # Read and scale/apply direction to wheels
        ids = [self.ID_FRONT_LEFT, self.ID_BACK_LEFT, self.ID_FRONT_RIGHT, self.ID_BACK_RIGHT]
        wheel_velocity = self.servo.read_velocity(ids)
        if wheel_velocity:
            wheel_velocity[0] *= self.DIR_FRONT_LEFT * 0.229
            wheel_velocity[1] *= self.DIR_BACK_LEFT * 0.229
            wheel_velocity[2] *= self.DIR_FRONT_RIGHT * 0.229
            wheel_velocity[3] *= self.DIR_BACK_RIGHT * 0.229
        else:
            wheel_velocity = []

        # Publish measured velocity
        measured_velocity_msg = Float32MultiArray()
        measured_velocity_msg.data = wheel_velocity
        self.measured_velocity_publisher.publish(measured_velocity_msg)

        # Read and scale current consumption
        wheel_current = self.servo.read_velocity(ids)
        if wheel_current:
            wheel_current = [curr * 2.69e-3 for curr in wheel_current]
        else:
            wheel_current = []

        # Publish current consumption
        current_consumption_msg = Float32MultiArray()
        current_consumption_msg.data = wheel_current
        self.current_consumption_publisher.publish(current_consumption_msg)


def main(args=None):
    rclpy.init(args=args)
    goat_controller_node = GoatController()
    rclpy.spin(goat_controller_node)
    goat_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
