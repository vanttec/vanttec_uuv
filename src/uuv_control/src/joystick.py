#!/usr/bin/env python3

# Original code from: https://github.com/DemianMArin/my_vanttec_uuv.git
# Modified by: Abraham de Jesus Maldonado Mata
# Date: 06/02/2025

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64MultiArray

import numpy as np
# import math

class TeleopControl(Node):

    def __init__(self):
        super().__init__('teleop_control_node')
        self.jsub = self.create_subscription(Joy, '/joy', self.convert_joy, 10)

        self.publisher = self.create_publisher(Float64MultiArray, 'uuv/forces', 10)
        self.vel_msg = Pose()

        self.multiplicador_tras = 1
        self.multiplicador_rota = 1

        self.joy = Float64MultiArray()
        self.joy.data = [0.0]*6
    
    def convert_joy(self, msg):
        surge = 1
        sway = 0
        heave_r = 5
        heave_l = 2

        pitch = 4
        roll = 3
        roll_p = 10
        roll_n = 9
        yaw = 3

        LB = msg.buttons[4]
        
        self.joy.data[0] = float(msg.axes[surge] * self.multiplicador_tras) # listo
        self.joy.data[1] = float(-msg.axes[sway] * self.multiplicador_tras) # listo
        self.joy.data[2] = float(-(msg.axes[heave_r]- msg.axes[heave_l])/2 * self.multiplicador_tras)
        self.joy.data[3] = float(msg.axes[pitch] * self.multiplicador_rota)# - msg.buttons[roll_n])
        self.joy.data[4]= float(-msg.axes[roll] * self.multiplicador_rota) if not(LB) else 0
        self.joy.data[5] = float(-msg.axes[yaw] * self.multiplicador_rota) if LB else 0 # listo

        self.publisher.publish(self.joy)

def main(args=None):
    rclpy.init(args=args)
    rmn = TeleopControl()
    rclpy.spin(rmn)
    rmn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()