#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import numpy as np
import pandas as pd
from pyntcloud import PyntCloud

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

from mixed_nav.srv import AdjustPcdZ, AdjustPcdZResponse

class PcdZAdjuster:
    def __init__(self):
        """
        节点初始化
        """
        rospy.loginfo("正在初始化PCD Z轴调整节点...")

        # 1. 获取ROS参数
        self.pcd_file_path = rospy.get_param('~pcd_file_path', '')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        topic_name = rospy.get_param('~topic_name', '/pointcloud_map')
        max_points = rospy.get_param('~max_points_before_downsample', 2000000)
        leaf_size = rospy.get_param('~leaf_size', 0.1)
        initial_z_offset = rospy.get_param('~initial_z_offset', 0.0)

        if not self.pcd_file_path or not os.path.exists(self.pcd_file_path):
            rospy.logerr(f"参数 'pcd_file_path' 未设置或文件不存在: {self.pcd_file_path}")
            return

        # 2. 加载和处理点云 (只在初始化时执行一次)
        self.base_points_np = self._load_and_process_pcd(self.pcd_file_path, max_points, leaf_size)
        if self.base_points_np is None:
            rospy.logerr("点云加载和处理失败，节点将退出。")
            return

        # 3. 创建发布者和服务
        self.pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1, latch=True)
        self.server = rospy.Service('/adjust_pcd_z_value', AdjustPcdZ, self.handle_adjust_z_service)
        
        rospy.loginfo(f"服务 '/adjust_pcd_z_value' 已就绪.")
        rospy.loginfo(f"将在主题 '{topic_name}' 上发布点云地图 (frame_id: {self.frame_id}).")

        # 4. 发布初始点云
        rospy.loginfo(f"使用默认Z轴偏移量 {initial_z_offset} 发布初始点云.")
        self._publish_and_save_cloud(initial_z_offset, save_file=False) # 初始发布时不保存文件

    def _load_and_process_pcd(self, file_path, max_points, leaf_size):
        """
        从文件中加载点云，如果点数过多则进行下采样。
        返回一个Numpy数组。
        """
        rospy.loginfo(f"正在从: {file_path} 加载PCD地图")
        try:
            cloud = PyntCloud.from_file(file_path)
        except Exception as e:
            rospy.logerr(f"使用 pyntcloud 加载PCD文件失败: {e}")
            return None

        point_count = len(cloud.points)
        rospy.loginfo(f"加载了包含 {point_count} 个点的点云。")

        if point_count > max_points:
            rospy.loginfo(f"点云数量 ({point_count}) 超出限制 ({max_points})，正在进行体素下采样...")
            voxelgrid_id = cloud.add_structure("voxelgrid", size_x=leaf_size, size_y=leaf_size, size_z=leaf_size)
            sampled_cloud_df = cloud.get_sample("voxelgrid_centroids", voxelgrid_id=voxelgrid_id)
            rospy.loginfo(f"下采样后剩余 {len(sampled_cloud_df)} 个点。")
            return sampled_cloud_df[['x', 'y', 'z']].values
        else:
            rospy.loginfo("点云数量在限制范围内，无需下采样。")
            return cloud.points[['x', 'y', 'z']].values

    def handle_adjust_z_service(self, req):
        """
        服务回调函数，处理Z轴调整请求。
        """
        new_z_value = req.z_value
        rospy.loginfo(f"接收到服务调用请求，将Z轴绝对高度调整为: {new_z_value}")

        try:
            self._publish_and_save_cloud(new_z_value, save_file=True)
            message = f"成功将点云Z轴调整为 {new_z_value} 并已发布和保存。"
            rospy.loginfo(message)
            return AdjustPcdZResponse(success=True, message=message)
        except Exception as e:
            message = f"处理服务请求时发生错误: {e}"
            rospy.logerr(message)
            return AdjustPcdZResponse(success=False, message=message)

    def _publish_and_save_cloud(self, z_offset, save_file=True):
        """
        调整点云的Z值，发布它，并根据标志选择是否保存到文件。
        """
        # 创建一个原始点云的副本进行操作，以避免修改原始数据
        adjusted_points_np = self.base_points_np.copy()
        
        # 将z_offset直接加到所有点的z值上
        adjusted_points_np[:, 2] += z_offset

        # 1. 发布调整后的点云
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        ros_msg = point_cloud2.create_cloud_xyz32(header, adjusted_points_np)
        self.pub.publish(ros_msg)
        rospy.loginfo("已发布调整后的点云。")

        # 2. 保存调整后的点云到临时文件
        if save_file:
            path_without_ext, ext = os.path.splitext(self.pcd_file_path)
            temp_file_path = f"{path_without_ext}_tmp{ext}"
            
            rospy.loginfo(f"正在将调整后的点云保存到: {temp_file_path}")
            
            # 使用PyntCloud保存文件
            df = pd.DataFrame(adjusted_points_np, columns=['x', 'y', 'z'])
            cloud_to_save = PyntCloud(df)
            cloud_to_save.to_file(temp_file_path)
            rospy.loginfo("文件保存成功。")


def main():
    rospy.init_node('pcd_z_adjuster_node', anonymous=True)
    try:
        adjuster = PcdZAdjuster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动时发生未处理的异常: {e}")

if __name__ == '__main__':
    main()