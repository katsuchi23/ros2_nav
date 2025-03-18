#!/usr/bin/env python3
# coding=utf8

import copy
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformBroadcaster
import transforms3d
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None
cur_map_to_odom = None


def pose_to_mat(pose_msg):
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    
    # Convert position to 4x4 matrix
    trans_mat = np.eye(4)
    trans_mat[0, 3] = position.x
    trans_mat[1, 3] = position.y
    trans_mat[2, 3] = position.z
    
    # Convert quaternion to rotation matrix and then to 4x4 matrix
    rot_mat = transforms3d.quaternions.quat2mat([orientation.w, orientation.x, orientation.y, orientation.z])
    rot4x4 = np.eye(4)
    rot4x4[:3, :3] = rot_mat
    
    # Combine translation and rotation
    return np.matmul(trans_mat, rot4x4)


class TransformFusionNode(Node):
    def __init__(self):
        super().__init__('transform_fusion')
        self.get_logger().info('Transform Fusion Node Inited...')
        
        # tf and localization publishing frequency (HZ)
        self.FREQ_PUB_LOCALIZATION = 50
        
        # Initialize subscribers and publishers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.cb_save_cur_odom,
            1)
            
        self.map_to_odom_sub = self.create_subscription(
            Odometry,
            '/map_to_odom',
            self.cb_save_map_to_odom,
            1)
            
        self.pub_localization = self.create_publisher(
            Odometry,
            '/localization',
            1)
            
        self.br = TransformBroadcaster(self)
        
        # Start transform fusion thread
        self.fusion_thread = threading.Thread(target=self.transform_fusion)
        self.fusion_thread.daemon = True
        self.fusion_thread.start()
    
    def transform_fusion(self):
        global cur_odom_to_baselink, cur_map_to_odom
        
        while rclpy.ok():
            time.sleep(1 / self.FREQ_PUB_LOCALIZATION)
            
            # Note on thread safety
            cur_odom = copy.copy(cur_odom_to_baselink)
            if cur_map_to_odom is not None:
                T_map_to_odom = pose_to_mat(cur_map_to_odom)
            else:
                T_map_to_odom = np.eye(4)
            
            # Broadcast transform using ROS2 method
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'camera_init'
            
            # Extract translation and rotation
            translation = transforms3d.affines.decompose44(T_map_to_odom)[0]
            rotation = transforms3d.quaternions.mat2quat(T_map_to_odom[:3, :3])
            
            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]
            t.transform.rotation.w = rotation[0]
            t.transform.rotation.x = rotation[1]
            t.transform.rotation.y = rotation[2]
            t.transform.rotation.z = rotation[3]
            
            self.br.sendTransform(t)
            
            if cur_odom is not None:
                # Publish global localization odometry
                localization = Odometry()
                T_odom_to_base_link = pose_to_mat(cur_odom)
                # T_map_to_odom changes slowly over time, so we don't consider time sync with T_odom_to_base_link
                T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)
                
                # Extract translation and rotation
                xyz = transforms3d.affines.decompose44(T_map_to_base_link)[0]
                quat = transforms3d.quaternions.mat2quat(T_map_to_base_link[:3, :3])
                
                localization.pose.pose = Pose(
                    Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                    Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
                )
                localization.twist = cur_odom.twist
                
                localization.header.stamp = cur_odom.header.stamp
                localization.header.frame_id = 'map'
                localization.child_frame_id = 'body'
                
                self.pub_localization.publish(localization)

    def cb_save_cur_odom(self, odom_msg):
        global cur_odom_to_baselink
        cur_odom_to_baselink = odom_msg

    def cb_save_map_to_odom(self, odom_msg):
        global cur_map_to_odom
        cur_map_to_odom = odom_msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
