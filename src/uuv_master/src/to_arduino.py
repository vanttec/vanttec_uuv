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

x_axes_i = 1        # [-1, 1]   LEFT STICK UP/DOWN
y_axes_i = 0        # [1, -1]   LEFT STICK LEFT/RIGHT
z_axes_neg_i = 2    # [1, -1]   LT
z_axes_pos_i = 5    # [1, -1]   RT
yaw_axes_i = 3      # [1, -1]   RIGHT STICK LEFT/RIGHT
pitch_axes_i = 4    # [-1, 1]   RIGHT STICK UP/DOWN

roll_buttons_neg_i = 4  # {0, 1}    LB
roll_buttons_pos_i = 5  # {0, 1}    RB

A = 0
X = 3
Y = 2
B = 1

top_left = 0
top_right = 1
middle_left = 2
middle_right = 3
bottom_left = 4
bottom_right = 5

# positions = [top_left, top_right, middle_left, middle_right, bottom_left, bottom_right]
values = ['','','','','','']

class ToArduinoNode(Node):
    def __init__(self):
        super().__init__('to_arduino')

        self.joy_sub_ = self.create_subscription(
            Joy, '/joy', self.get_joy, 10
        )

        self.serial_pub_ = self.create_publisher(Float64MultiArray, 'arduino_read', 10)

        self.ser = serial.Serial(arduinoPort, 9600, timeout=1)
        self.get_logger().info('Serial port initialized')

    def get_joy(self, msg):
        x = msg.axes[x_axes_i]
        y = msg.axes[y_axes_i]
        x_map = map_range(float(x), -1, 1, 1100, 1900)
        y_map = map_range(float(y), -1, 1, 1100, 1900)
        # x = map_range(float(msg.axes[x_axes_i]), -1, 1, 1100, 1900)
        # y = map_range(float(msg.axes[y_axes_i]), -1, 1, 1100, 1900)
        for i in range(6):
            values[i] = '1100'
        
        if x<=0.000001 and np.abs(y) <= 0.5:
            x_map = str(map_range(float(x), -1, 0, 1900, 1100))
            values[middle_left] = x_map
            values[middle_right] = x_map
            values[bottom_left] = x_map
            values[bottom_right] = x_map
        elif x>=0.000001 and np.abs(y) <= 0.5:
            x_map = str(map_range(float(x), 0, 1, 1100, 1900))
            values[middle_left] = x_map
            values[middle_right] = x_map
            values[top_left] = x_map
            values[top_right] = x_map
        elif y>=0.000001 and np.abs(x) <= 0.5:
            y_map = str(map_range(float(y), 0, 1, 1100, 1900))
            values[middle_left] = y_map
            values[top_left] = y_map
            values[bottom_left] = y_map
        elif y<=0.000001 and np.abs(x) <= 0.5:
            y_map = str(map_range(float(y), -1, 0, 1900, 1100))
            values[middle_right] = y_map
            values[top_right] = y_map
            values[bottom_right] = y_map
        
        #self.get_logger().info(str(x_map))

        if msg.buttons[A]:
            values[middle_left] = '1900'
            values[middle_right] = '1900'
            values[bottom_left] = '1900'
            values[bottom_right] = '1900'
        elif msg.buttons[X]:
            values[top_left] = '1900'
            values[middle_left] = '1900'
            values[bottom_left] = '1900'
        elif msg.buttons[Y]:
            values[top_left] = '1900'
            values[top_right] = '1900'
            values[middle_left] = '1900'
            values[middle_right] = '1900'
        elif msg.buttons[B]:
            values[top_right] = '1900'
            values[middle_right] = '1900'
            values[bottom_right] = '1900'
            

        toSend = '(' + values[0]
        self.get_logger().info(f"Motor {1}: {values[0]}")
        for i in range(5):
            toSend = toSend + ',' + values[i+1]
            self.get_logger().info(f"Motor {i+2}: {values[i+1]}")
        
        toSend += ')'
        toSend = f"{toSend}\n"
        # self.get_logger().info(toSend)
        self.ser.write(toSend.encode())

def map_range(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def main(args=None):
    rclpy.init(args=args)
    node = ToArduinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
