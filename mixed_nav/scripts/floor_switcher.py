#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os
import json

from mixed_nav.srv import AdjustPcdZ
from mixed_nav.srv import SwitchPath, SwitchFloor, SwitchFloorResponse, Switch2DMap

class FloorSwitcher:
    def __init__(self):
        """
        节点初始化
        """
        rospy.loginfo("正在初始化楼层切换总控节点...")

        # 1. 找到包含资源文件的包路径 (我们假设是 mixed_nav 包)
        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('mixed_nav') 
            self.config_path = os.path.join(package_path, 'resources', 'floors', 'z_config.json')
            rospy.loginfo(f"z_config.json 路径已设定: {self.config_path}")
        except rospkg.ResourceNotFound:
            rospy.logerr("错误: 未找到 'mixed_nav' 包。无法定位 z_config.json。")
            return

        try:
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('mixed_nav') 
            self.config_path = os.path.join(package_path, 'resources', 'floors', 'z_config.json')
        except rospkg.ResourceNotFound:
            rospy.logerr("错误: 未找到 'mixed_nav' 包。")
            return

        # 2. 等待并连接到所有依赖的服务
        try:
            rospy.loginfo("正在等待服务 /adjust_pcd_z_value...")
            rospy.wait_for_service('/adjust_pcd_z_value', timeout=30.0)
            self.adjust_pcd_client = rospy.ServiceProxy('/adjust_pcd_z_value', AdjustPcdZ)

            rospy.loginfo("正在等待服务 /switch_path...")
            rospy.wait_for_service('/switch_path', timeout=30.0)
            self.switch_path_client = rospy.ServiceProxy('/switch_path', SwitchPath)

            # 新增：等待动态地图加载服务
            rospy.loginfo("正在等待服务 /switch_2d_map...")
            rospy.wait_for_service('/switch_2d_map', timeout=30.0)
            self.switch_2d_map_client = rospy.ServiceProxy('/switch_2d_map', Switch2DMap)
            
            rospy.loginfo("所有依赖的服务已连接。")

        except rospy.ROSException as e:
            rospy.logerr(f"依赖的服务未在30秒内启动，节点将退出: {e}")
            return

        # 3. 创建并提供 /switch_floor 服务
        self.switch_service = rospy.Service('/switch_floor', SwitchFloor, self.handle_switch_floor)
        rospy.loginfo("服务 /switch_floor 已就绪。")


    def handle_switch_floor(self, req):
        target_floor = req.floor_name
        rospy.loginfo(f"收到切换到楼层 '{target_floor}' 的请求...")

        try:
            # --- 步骤 1: 读取配置 ---
            with open(self.config_path, 'r') as f:
                config_data = json.load(f)
            if target_floor not in config_data:
                raise ValueError(f"在 z_config.json 中未找到楼层 '{target_floor}'。")
            z_offset = config_data[target_floor].get('z_min')
            if z_offset is None:
                raise ValueError(f"楼层 '{target_floor}' 配置中缺少 'z_min'。")
            rospy.loginfo(f"找到 '{target_floor}' 的 z_min = {z_offset}。")

            # --- 步骤 2: 调整3D点云地图 ---
            pcd_response = self.adjust_pcd_client(z_value=z_offset)
            if not pcd_response.success:
                raise RuntimeError(f"调用 /adjust_pcd_z_value 失败: {pcd_response.message}")
            rospy.loginfo("3D点云地图Z轴调整成功。")

            # --- 步骤 3: 切换导航路径 ---
            path_response = self.switch_path_client(path_key=target_floor)
            if not path_response.success:
                raise RuntimeError(f"调用 /switch_path 失败: {path_response.message}")
            rospy.loginfo("导航路径切换成功。")
            
            # --- 新增步骤 4: 切换2D栅格地图 ---
            rospy.loginfo(f"正在调用 /switch_2d_map 服务，切换到地图 '{target_floor}'...")
            map_response = self.switch_2d_map_client(floor_name=target_floor)
            if not map_response.success:
                 raise RuntimeError(f"调用 /switch_2d_map 失败: {map_response.message}")
            rospy.loginfo("2D栅格地图切换成功。")

            # --- 所有步骤成功 ---
            final_message = f"完全切换到楼层 '{target_floor}' 成功。"
            rospy.loginfo(final_message)
            return SwitchFloorResponse(success=True, message=final_message)

        except Exception as e:
            error_message = f"切换到楼层 '{target_floor}' 失败: {e}"
            rospy.logerr(error_message)
            return SwitchFloorResponse(success=False, message=error_message)


def main():
    rospy.init_node('floor_switcher_node')
    try:
        switcher = FloorSwitcher()
        rospy.spin() # 保持节点运行以响应服务调用
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()