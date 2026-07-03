#!/usr/bin/env python3
"""Mock sensor publishers for integration testing of oni_nav_controller."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from oni_msgs.msg import BBox, SystemState


class MockNavInputs(Node):
    def __init__(self):
        super().__init__('mock_nav_inputs')
        self.declare_parameter('mode', 'chase')
        self.mode = self.get_parameter('mode').value

        self.bbox_pub = self.create_publisher(BBox, '/perception/bbox', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 10)
        self.state_pub = self.create_publisher(SystemState, '/system/state', 10)

        self.t = 0.0
        self.timer = self.create_timer(0.05, self.publish_all)
        self.get_logger().info(f'MockNavInputs started (mode={self.mode})')

    def publish_all(self):
        self.t += 0.05
        stamp = self.get_clock().now().to_msg()

        bbox = BBox()
        bbox.stamp = stamp
        bbox.u = 320.0
        bbox.h = 80.0
        bbox.area_ratio = 0.15
        bbox.lost = self.mode == 'lost'
        self.bbox_pub.publish(bbox)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 0.0
        self.imu_pub.publish(imu)

        state = SystemState()
        state.stamp = stamp
        if self.mode == 'idle':
            state.state = SystemState.IDLE
        elif self.mode == 'lost':
            state.state = SystemState.LOST
        elif self.mode == 'stop':
            state.state = SystemState.STOP
        else:
            state.state = SystemState.CHASE
        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = MockNavInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
