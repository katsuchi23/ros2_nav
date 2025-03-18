#!/usr/bin/env python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
import numpy as np
from tf_transformations import translation_from_matrix, quaternion_from_matrix
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('fast_lio_localization')
        
        self.global_map = None
        self.initialized = False
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None

        # Parameters
        self.MAP_VOXEL_SIZE = 0.4
        self.SCAN_VOXEL_SIZE = 0.1

        # Global localization frequency (HZ)
        self.FREQ_LOCALIZATION = 0.5

        # The threshold of global localization,
        # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
        self.LOCALIZATION_TH = 0.95

        # FOV(rad), modify this according to your LiDAR type
        self.FOV = 1.6

        # The farthest distance(meters) within FOV
        self.FOV_FAR = 150

        # Publishers
        self.pub_pc_in_map = self.create_publisher(PointCloud2, '/cur_scan_in_map', 1)
        self.pub_submap = self.create_publisher(PointCloud2, '/submap', 1)
        self.pub_map_to_odom = self.create_publisher(Odometry, '/map_to_odom', 1)

        # Subscriptions
        self.create_subscription(PointCloud2, '/cloud_registered', self.cb_save_cur_scan, 1)
        self.create_subscription(Odometry, '/Odometry', self.cb_save_cur_odom, 1)

        self.get_logger().info('Localization Node Inited...')

        # Initialize global map
        self.get_logger().warn('Waiting for global map......')
        # Wait for the first message - handled in main function

    def pose_to_mat(self, pose_msg):
        # Using tf_transformations to create matrices
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation
        
        # Create translation matrix
        trans_mat = np.eye(4)
        trans_mat[0, 3] = position.x
        trans_mat[1, 3] = position.y
        trans_mat[2, 3] = position.z
        
        # Create rotation matrix
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        rot_mat = np.eye(4)
        rot_mat[:3, :3] = np.array(quaternion_matrix(q)[:3, :3])
        
        # Multiply them
        return np.matmul(trans_mat, rot_mat)

    def msg_to_array(self, pc_msg):
        pc_array = ros_numpy.numpify(pc_msg)
        pc = np.zeros([len(pc_array), 3])
        pc[:, 0] = pc_array['x']
        pc[:, 1] = pc_array['y']
        pc[:, 2] = pc_array['z']
        return pc

    def registration_at_scale(self, pc_scan, pc_map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
            self.voxel_down_sample(pc_scan, self.SCAN_VOXEL_SIZE * scale), 
            self.voxel_down_sample(pc_map, self.MAP_VOXEL_SIZE * scale),
            1.0 * scale, initial,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20)
        )

        return result_icp.transformation, result_icp.fitness

    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse

    def publish_point_cloud(self, publisher, header, pc):
        data = np.zeros(len(pc), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        data['x'] = pc[:, 0]
        data['y'] = pc[:, 1]
        data['z'] = pc[:, 2]
        if pc.shape[1] == 4:
            data['intensity'] = pc[:, 3]
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header = header
        publisher.publish(msg)

    def crop_global_map_in_FOV(self, global_map, pose_estimation, cur_odom):
        # 当前scan原点的位姿
        T_odom_to_base_link = self.pose_to_mat(cur_odom)
        T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
        T_base_link_to_map = self.inverse_se3(T_map_to_base_link)

        # 把地图转换到lidar系下
        global_map_in_map = np.array(global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

        # 将视角内的地图点提取出来
        if self.FOV > 3.14:
            # 环状lidar 仅过滤距离
            indices = np.where(
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        else:
            # 非环状lidar 保前视范围
            # FOV_FAR>x>0 且角度小于FOV
            indices = np.where(
                (global_map_in_base_link[:, 0] > 0) &
                (global_map_in_base_link[:, 0] < self.FOV_FAR) &
                (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < self.FOV / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        # 发布fov内点云
        header = cur_odom.header
        header.frame_id = 'map'
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        # 用icp配准
        self.get_logger().info('Global localization by scan-to-map matching......')

        # TODO 这里注意线程安全
        scan_tobe_mapped = copy.copy(self.cur_scan)

        tic = time.time()

        global_map_in_FOV = self.crop_global_map_in_FOV(self.global_map, pose_estimation, self.cur_odom)

        # 粗配准
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

        # 精配准
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                        scale=1)
        toc = time.time()
        self.get_logger().info('Time: {}'.format(toc - tic))
        self.get_logger().info('')

        # 当全局定位成功时才更新map2odom
        if fitness > self.LOCALIZATION_TH:
            # T_map_to_odom = np.matmul(transformation, pose_estimation)
            self.T_map_to_odom = transformation

            # 发布map_to_odom
            map_to_odom = Odometry()
            xyz = translation_from_matrix(self.T_map_to_odom)
            quat = quaternion_from_matrix(self.T_map_to_odom)
            map_to_odom.pose.pose = Pose(Point(x=xyz[0], y=xyz[1], z=xyz[2]), 
                                        Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))
            map_to_odom.header.stamp = self.cur_odom.header.stamp
            map_to_odom.header.frame_id = 'map'
            self.pub_map_to_odom.publish(map_to_odom)
            return True
        else:
            self.get_logger().warn('Not match!!!!')
            self.get_logger().warn('{}'.format(transformation))
            self.get_logger().warn('fitness score:{}'.format(fitness))
            return False

    def voxel_down_sample(self, pcd, voxel_size):
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        except:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
        return pcd_down

    def initialize_global_map(self, pc_msg):
        self.global_map = o3d.geometry.PointCloud()
        self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = self.voxel_down_sample(self.global_map, self.MAP_VOXEL_SIZE)
        self.get_logger().info('Global map received.')

    def cb_save_cur_odom(self, odom_msg):
        self.cur_odom = odom_msg

    def cb_save_cur_scan(self, pc_msg):
        # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
        pc_msg.header.frame_id = 'camera_init'
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_pc_in_map.publish(pc_msg)

        # 转换为pcd
        # fastlio给的field有问题 处理一下
        pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                        pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                        pc_msg.fields[3], pc_msg.fields[7]]
        pc = self.msg_to_array(pc_msg)

        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

    def thread_localization(self):
        while rclpy.ok():
            # 每隔一段时间进行全局定位
            time.sleep(1 / self.FREQ_LOCALIZATION)
            # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
            self.global_localization(self.T_map_to_odom)


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion."""
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < np.finfo(float).eps:
        return np.identity(4)
    q *= np.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0],
        [    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0],
        [    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalization()
    
    # 初始化全局地图
    global_map_msg = None
    
    # Wait for global map message
    while global_map_msg is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
        try:
            global_map_subscription = node.create_subscription(PointCloud2, '/map', 
                                                             lambda msg: globals().update({'global_map_msg': msg}), 1)
            node.get_logger().warn('Waiting for global map...')
            time.sleep(1.0)
        except Exception as e:
            node.get_logger().error(f"Error waiting for map: {e}")
    
    if global_map_msg:
        node.initialize_global_map(global_map_msg)
    
    # 初始化
    while not node.initialized and rclpy.ok():
        node.get_logger().warn('Waiting for initial pose....')
        
        # Wait for initial pose 
        pose_msg = None
        initial_pose_subscription = node.create_subscription(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            lambda msg: globals().update({'pose_msg': msg}), 
            1)
        
        while pose_msg is None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        
        if pose_msg:
            initial_pose = node.pose_to_mat(pose_msg)
            if node.cur_scan:
                node.initialized = node.global_localization(initial_pose)
            else:
                node.get_logger().warn('First scan not received!!!!!')
    
    if node.initialized:
        node.get_logger().info('')
        node.get_logger().info('Initialize successfully!!!!!!')
        node.get_logger().info('')
        
        # 开始定期全局定位
        localization_thread = threading.Thread(target=node.thread_localization)
        localization_thread.daemon = True
        localization_thread.start()
        
        rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
