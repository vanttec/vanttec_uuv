#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('sub_vis')
        self.posicion_ = self.create_subscription(Pose, 'uuv/state/pose', self.pose_objetivo_callback, 10)
        self.publisher_ = self.create_publisher(Marker, 'sub_topic', 10)

        self.pose_objetivo_ = [0]*7


        self.timer = self.create_timer(0.1, self.publish_sub)


    def publish_sub(self):
        sub = Marker()
        sub.header.frame_id = "map"
        sub.header.stamp = self.get_clock().now().to_msg()
        sub.ns = "example_namespace"
        sub.id = 0
        #sub.type = Marker.MESH_RESOURCE
        #sub.mesh_resource = "package://sub/models/uuv.STL"
        #sub.mesh_resource = "package://turtlebot3_description/meshes/bases/burger_base.stl"
        sub.type = Marker.CUBE
        sub.action = Marker.ADD


        sub.pose.position.x = float(self.pose_objetivo_[0])
        sub.pose.position.y = float(self.pose_objetivo_[1])
        sub.pose.position.z = float(self.pose_objetivo_[2])
        sub.pose.orientation.x = float(self.pose_objetivo_[3])
        sub.pose.orientation.y = float(self.pose_objetivo_[4])
        sub.pose.orientation.z = float(self.pose_objetivo_[5])
        sub.pose.orientation.w = float(self.pose_objetivo_[6])
        sub.scale.x = 0.5
        sub.scale.y = 0.5
        sub.scale.z = 0.5
        sub.color.a = 1.0  # Alpha value
        sub.color.r = 0.0
        sub.color.g = 1.0
        sub.color.b = 0.0

        self.publisher_.publish(sub)
        self.get_logger().info('Publishing sub')

    def pose_objetivo_callback(self, msg):
        self.pose_objetivo_[0] = msg.position.x
        self.pose_objetivo_[1] = msg.position.y
        self.pose_objetivo_[2] = msg.position.z
        self.pose_objetivo_[3] = msg.orientation.x
        self.pose_objetivo_[4] = msg.orientation.y
        self.pose_objetivo_[5] = msg.orientation.z
        self.pose_objetivo_[6] = msg.orientation.w


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
