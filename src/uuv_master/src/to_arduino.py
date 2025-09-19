#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

arduinoPort = '/dev/ttyACM0'

class ToArduinoNode(Node):
    def __init__(self):
        super().__init__('to_arduino')

        self.joy_sub_ = self.create_subscription(
            Joy, '/joy', self.get_joy, 10
        )

        self.serial_pub_ = self.create_publisher(Float64MultiArray, 'arduino_read', 10)

        self.ser = serial.Serial(arduinoPort, 9600, timeout=1)
        self.get_logger().info('Serial port initialized')

        self.x_axes_i = 1        # [-1, 1]   LEFT STICK UP/DOWN

    def get_joy(self, msg):
        x = (msg.axes[self.x_axes_i])
        x = int(x * 100)
        x_str = f"{x}\n"
        self.ser.write(x_str.encode())
        self.get_logger().info(f'Sent to Arduino: {x_str.strip()}')





def main(args=None):
    rclpy.init(args=args)
    node = ToArduinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
