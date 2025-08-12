#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pyntcloud import PyntCloud

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

def main():
    # 1. 初始化ROS节点
    rospy.init_node('pcd_map_publisher_pynt', anonymous=True)
    # 2. 从私有参数服务器获取参数
    pcd_file_path = rospy.get_param('~pcd_file_path', '')
    frame_id = rospy.get_param('~frame_id', 'map')
    topic_name = rospy.get_param('~topic_name', '/pointcloud_map')
    max_points = rospy.get_param('~max_points_before_downsample', 2000000)
    leaf_size = rospy.get_param('~leaf_size', 0.1)

    if not pcd_file_path:
        rospy.logerr("Parameter 'pcd_file_path' is not set! Please provide the absolute path to the .pcd file.")
        return

    rospy.loginfo("Loading PCD map from: %s using pyntcloud", pcd_file_path)
    rospy.loginfo("Publishing on topic: %s with frame_id: %s", topic_name, frame_id)

    # 3. 创建点云发布者 (锁存话题)
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1, latch=True)
    # 4. 使用 pyntcloud 加载PCD文件
    try:
        cloud = PyntCloud.from_file(pcd_file_path)
    except Exception as e:
        rospy.logerr("Failed to load PCD file with pyntcloud: %s", str(e))
        return
    
    point_count = len(cloud.points)
    rospy.loginfo("Loaded point cloud with %d points.", point_count)

    # 5. 检查是否需要下采样
    points_to_publish_np = None
    if point_count > max_points:
        rospy.loginfo("Point cloud is too large (%d points > %d). Downsampling...", point_count, max_points)
        
        # 使用VoxelGrid的质心进行采样，效果等同于下采样
        # 首先，需要为点云添加一个VoxelGrid结构
        voxelgrid_id = cloud.add_structure("voxelgrid", size_x=leaf_size, size_y=leaf_size, size_z=leaf_size)
        # 然后，从该结构中采样质心点
        sampled_cloud_df = cloud.get_sample("voxelgrid_centroids", voxelgrid_id=voxelgrid_id)
        rospy.loginfo("Downsampled to %d points.", len(sampled_cloud_df))
        # 从DataFrame中提取 x, y, z 列并转换为NumPy数组
        points_to_publish_np = sampled_cloud_df[['x', 'y', 'z']].values
    else:
        rospy.loginfo("Point cloud size is within the limit. No downsampling needed.")
        # 从原始点云的DataFrame中提取 x, y, z 列并转换为NumPy数组
        points_to_publish_np = cloud.points[['x', 'y', 'z']].values
    
    # 6. 将NumPy点云数据转换为ROS PointCloud2 消息
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    
    # 使用ROS的工具函数从NumPy数组创建消息
    ros_msg = point_cloud2.create_cloud_xyz32(header, points_to_publish_np)

    # 7. 发布消息
    pub.publish(ros_msg)
    rospy.loginfo("Point cloud map published. The node will keep running to latch the topic.")

    # 8. 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass