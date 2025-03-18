#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import argparse
import sys

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped

def main(args=None):
    if args is None:
        args = sys.argv[1:]
        
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('z', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    args = parser.parse_args(args)

    rclpy.init()
    node = Node('publish_initial_pose')
    pub_pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    # 转换为pose
    quat = quaternion_from_euler(args.roll, args.pitch, args.yaw)
    xyz = [args.x, args.y, args.z]

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose = Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]), 
                                  orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
    initial_pose.header.stamp = node.get_clock().now().to_msg()
    initial_pose.header.frame_id = 'map'
    
    # Sleep in ROS2
    rclpy.sleep(1.0)
    
    node.get_logger().info('Initial Pose: {} {} {} {} {} {}'.format(
        args.x, args.y, args.z, args.yaw, args.pitch, args.roll))
    pub_pose.publish(initial_pose)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
