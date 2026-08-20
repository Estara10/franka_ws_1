#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

def main():
    rclpy.init()
    node = Node('set_home_joint_state')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    msg = JointState()
    msg.name = [
        'panda_1_joint1', 'panda_1_joint2', 'panda_1_joint3', 'panda_1_joint4', 'panda_1_joint5', 'panda_1_joint6', 'panda_1_joint7',
        'panda_2_joint1', 'panda_2_joint2', 'panda_2_joint3', 'panda_2_joint4', 'panda_2_joint5', 'panda_2_joint6', 'panda_2_joint7'
    ]
    msg.position = [
        0.0, 0.6, 0.0, -1.4, 0.0, 1.0, 0.0,   # panda_1
        0.0, 0.6, 0.0, -1.4, 0.0, 1.0, 0.0    # panda_2
    ]
    msg.velocity = []
    msg.effort = []
    # 多次发布，确保被rviz/robot_state_publisher接收
    for _ in range(10):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
