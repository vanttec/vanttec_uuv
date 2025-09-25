#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import numpy as np

arduinoPort = '/dev/ttyACM0'
limitator = 0.6   # 1 = no limitation



class Limitator(Node):

    def __init__(self):
        super().__init__('limitator')

        self.joy_sub_ = self.create_subscription(
            Float64MultiArray, '/uuv/matriz_locacion', self.get_joy, 10
        )

        self.serial_pub_ = self.create_publisher(Float64MultiArray, 'arduino_read', 10)

        self.ser = serial.Serial(arduinoPort, 115200, timeout=1)
        self.get_logger().info('Serial port initialized')

    def get_joy(self, F):

        
        toSend = '(' + str(map_range(F.data[0], -1, 1, 1100/limitator, 1900*limitator))
        # self.get_logger().info(f"Motor {1}: {values[0]}")
        for i in range(5):
            toSend = toSend + ',' + str(map_range(F.data[i+1], -1, 1, 1100/limitator, 1900*limitator))
            # self.get_logger().info(f"Motor {i+2}: {values[i+1]}")
        
        toSend += ')'
        toSend = f"{toSend}\n"
        self.get_logger().info(toSend)
        self.ser.write(toSend.encode())

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def main(args=None):
    rclpy.init(args=args)
    node = Limitator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
