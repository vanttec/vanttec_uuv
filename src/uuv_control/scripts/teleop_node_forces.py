#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


x_axes_i = 1        # [-1, 1]   LEFT STICK UP/DOWN
y_axes_i = 0        # [1, -1]   LEFT STICK LEFT/RIGHT
z_axes_neg_i = 2    # [1, -1]   LT
z_axes_pos_i = 5    # [1, -1]   RT
yaw_axes_i = 3      # [1, -1]   RIGHT STICK LEFT/RIGHT
pitch_axes_i = 4    # [-1, 1]   RIGHT STICK UP/DOWN

roll_buttons_neg_i = 4  # {0, 1}    LB
roll_buttons_pos_i = 5  # {0, 1}    RB

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_node_forces')
        self.jsub_ = self.create_subscription(
            Joy, '/joy', self.convert_joy, 10
        )

        self.key_sub_ = self.create_subscription(
            Twist, '/cmd_vel', self.convert_key, 10
        )

        self.thrust_pub_ = self.create_publisher(Float64MultiArray, 'uuv/forces', 10)

        self.max_t = np.array([4., 2., 2., 0.25, 0.25, 0.25])
        # self.curr_t = [0.]*6
        # self.throttle_increment = 0.1
        # self.brake_increment = 0.1

    def convert_key(self, msg):
        # TODO
        pass

    def convert_joy(self, msg):
        x = (msg.axes[x_axes_i])
        y = (-msg.axes[y_axes_i])
        z = (-msg.axes[z_axes_pos_i]) - (-msg.axes[z_axes_neg_i])
        roll = msg.buttons[roll_buttons_pos_i] - msg.buttons[roll_buttons_neg_i]
        pitch = (msg.axes[pitch_axes_i])
        yaw = (msg.axes[yaw_axes_i])

        u = np.array([x, y, z, roll, pitch, yaw])
        # u = np.array([x, y, z, 0., 0., 0.])

        thrust_msg_ = Float64MultiArray()
        for i in range(6):
            if abs(u[i]) < 0.05:
                u[i] = 0.
            thrust_msg_.data.append(u[i]*self.max_t[i])

        self.thrust_pub_.publish(thrust_msg_)

def main(args=None):
    rclpy.init(args=args)
    rmn = TeleopControl()
    rclpy.spin(rmn)
    rmn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()