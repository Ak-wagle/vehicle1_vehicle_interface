import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
import serial
import time
import math

class MotorControlLogger(Node):
    def __init__(self):
        super().__init__('motor_control_logger')

        # Subscription - tire angle from control command
        self.subscription = self.create_subscription(
            Control,
            '/control/command/control_cmd',
            self.control_callback,
            10
        )

        # Publisher - steering report
        self.steering_report_pub = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', 10
        )

        # Serial Communication
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.serial_port = None

        # Timer to read encoder feedback
        self.create_timer(0.05, self.read_encoder_feedback)

    def control_callback(self, msg):
        if self.serial_port is None:
            self.get_logger().error("Serial port not initialized. Skipping message.")
            return

        # Radians to degrees convertion
        steering_angle_rad = msg.lateral.steering_tire_angle
        steering_angle_deg = math.degrees(steering_angle_rad)

        # Steering command to Arduino
        command = f"S:{steering_angle_deg:.2f}\n"
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent: {steering_angle_deg:.2f}°")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_encoder_feedback(self):
        if self.serial_port is None or not self.serial_port.in_waiting:
            return

        try:
            feedback = self.serial_port.readline().decode('utf-8').strip()
            if feedback.startswith("A:"):
                angle_deg = float(feedback[2:])

                report = SteeringReport()
                report.stamp = self.get_clock().now().to_msg()
                report.steering_tire_angle = math.radians(angle_deg)
                self.steering_report_pub.publish(report)

                self.get_logger().info(f"Received Angle: {angle_deg:.2f}°")
        except (ValueError, serial.SerialException) as e:
            self.get_logger().error(f"Error reading encoder feedback: {e}")

    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
            self.get_logger().info("Closed serial connection")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
