#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
import os
import yaml
import numpy as np
from PIL import Image

from nav_msgs.msg import OccupancyGrid
from mixed_nav.srv import Switch2DMap, Switch2DMapResponse

class DynamicMapLoader:
    def __init__(self):
        rospy.loginfo("正在初始化动态地图加载节点...")

        # 1. 找到包含资源文件的包路径
        try:
            rospack = rospkg.RosPack()
            self.base_path = rospack.get_path('mixed_nav')
            self.floors_dir = os.path.join(self.base_path, 'resources', 'floors')
        except rospkg.ResourceNotFound:
            rospy.logerr("错误: 未找到 'mixed_nav' 包。无法定位地图文件。")
            return

        # 2. 创建一个锁存的发布者，用于发布/map话题
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        
        # 3. 创建服务
        self.switch_map_service = rospy.Service('/switch_2d_map', Switch2DMap, self.handle_switch_map)
        
        # 4. 加载一个初始地图
        initial_map = rospy.get_param('~initial_map', None)
        if initial_map:
            self.load_and_publish_map(initial_map)
            
        rospy.loginfo("动态地图加载节点已就绪。")

    def handle_switch_map(self, req):
        success, message = self.load_and_publish_map(req.floor_name)
        return Switch2DMapResponse(success=success, message=message)

    def load_and_publish_map(self, floor_name):
        rospy.loginfo(f"收到加载楼层 '{floor_name}' 的2D地图请求...")
        
        try:
            # 1. 构建YAML文件路径
            yaml_path = os.path.join(self.floors_dir, floor_name, f"{floor_name}.yaml")
            if not os.path.exists(yaml_path):
                raise FileNotFoundError(f"地图配置文件不存在: {yaml_path}")

            # 2. 读取YAML文件
            with open(yaml_path, 'r') as f:
                map_config = yaml.safe_load(f)

            # 3. 构建PGM图像文件路径
            pgm_path = os.path.join(os.path.dirname(yaml_path), map_config['image'])
            if not os.path.exists(pgm_path):
                 raise FileNotFoundError(f"地图图像文件不存在: {pgm_path}")
            
            # 4. 加载图像并转换为Numpy数组
            img = Image.open(pgm_path)
            map_np = np.array(img, dtype=np.int8)

            # 5. 创建 OccupancyGrid 消息
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = rospy.Time.now()
            grid_msg.header.frame_id = 'map'
            
            # 填充地图元数据
            grid_msg.info.resolution = map_config['resolution']
            grid_msg.info.width = img.width
            grid_msg.info.height = img.height
            grid_msg.info.origin.position.x = map_config['origin'][0]
            grid_msg.info.origin.position.y = map_config['origin'][1]
            grid_msg.info.origin.position.z = map_config['origin'][2]
            # ROS地图的姿态通常是单位四元数
            grid_msg.info.origin.orientation.w = 1.0

            # 转换地图数据
            # PGM值 -> OccupancyGrid值
            # 0 (黑/占据) -> 100
            # 254 (白/空闲) -> 0
            # 205 (灰/未知) -> -1
            # 图像数据通常是(高, 宽)，且(0,0)在左上角，需要翻转Y轴
            map_data_flipped = np.flipud(map_np)
            
            # 使用Numpy高效转换
            ros_map_data = np.full(map_data_flipped.shape, -1, dtype=np.int8) # 默认未知
            ros_map_data[map_data_flipped == 0] = 100 # 占据
            ros_map_data[map_data_flipped >= 250] = 0   # 空闲 (处理254和255)
            
            grid_msg.data = ros_map_data.ravel().tolist()

            # 6. 发布地图
            self.map_pub.publish(grid_msg)
            
            message = f"成功加载并发布了 '{floor_name}' 的2D地图。"
            rospy.loginfo(message)
            return True, message

        except Exception as e:
            message = f"加载地图 '{floor_name}' 失败: {e}"
            rospy.logerr(message)
            return False, message

if __name__ == '__main__':
    rospy.init_node('dynamic_map_loader')
    try:
        loader = DynamicMapLoader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass