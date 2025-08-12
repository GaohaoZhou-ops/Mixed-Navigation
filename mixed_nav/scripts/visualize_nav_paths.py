#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import rospkg
import commentjson
import random

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from mixed_nav.srv import SwitchPath, SwitchPathResponse

class PathVisualizer:
    def __init__(self):
        """
        节点初始化
        """
        rospy.loginfo("正在初始化路径可视化与切换节点...")

        # 1. 初始化ROS包路径查找器
        try:
            self.rospack = rospkg.RosPack()
            self.package_path = self.rospack.get_path('mixed_nav')
            rospy.loginfo(f"成功找到 'mixed_nav' 包，路径: {self.package_path}")
        except rospkg.ResourceNotFound:
            rospy.logerr("错误: 未找到名为 'mixed_nav' 的ROS包。请确保该包在您的工作空间中。")
            return

        # 2. 初始化发布者和当前要显示的MarkerArray
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10, latch=True)
        self.current_marker_array = None

        # 3. 创建服务
        self.switch_service = rospy.Service('/switch_path', SwitchPath, self.handle_switch_path)
        rospy.loginfo("服务 '/switch_path' 已就绪.")

        # 4. 加载初始路径（如果参数已设置）
        initial_path_key = rospy.get_param('~initial_path_key', None) # e.g., "floor0"
        if initial_path_key:
            rospy.loginfo(f"正在加载初始路径: {initial_path_key}")
            self.load_and_publish_path(initial_path_key)
        else:
            rospy.loginfo("未设置初始路径，等待服务调用...")

        # 5. 设置一个定时器来周期性地发布MarkerArray，以保持其在RViz中可见
        self.publish_timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def handle_switch_path(self, req):
        """
        服务回调函数，处理路径切换请求
        """
        path_key = req.path_key
        rospy.loginfo(f"收到服务请求，切换路径到: '{path_key}'")
        
        success = self.load_and_publish_path(path_key)
        
        if success:
            message = f"成功加载并发布路径 '{path_key}'."
            return SwitchPathResponse(success=True, message=message)
        else:
            message = f"加载路径 '{path_key}' 失败。请检查文件名和日志。"
            return SwitchPathResponse(success=False, message=message)

    def load_and_publish_path(self, path_key):
        """
        核心函数：构建路径，加载文件，创建Markers并发布
        """
        # 1. 清除旧的Markers
        self._clear_all_markers()
        
        # 2. 构建文件路径: [package_path]/resources/floors/[path_key]/waypoints.json
        file_path = os.path.join(self.package_path, 'resources', 'floors', path_key, 'waypoints.json')
        
        if not os.path.exists(file_path):
            rospy.logerr(f"文件不存在: {file_path}")
            self.current_marker_array = None # 清空当前路径
            return False

        # 3. 加载导航点
        nav_point_groups = self._load_nav_points(file_path)
        if not nav_point_groups:
            self.current_marker_array = None # 清空当前路径
            return False

        # 4. 创建新的MarkerArray
        marker_array = MarkerArray()
        marker_id_counter = 0
        for group_name, points in nav_point_groups.items():
            color = self._generate_random_color()
            path_marker = self._create_path_marker(group_name, points, marker_id_counter, color)
            marker_array.markers.append(path_marker)
            marker_id_counter += 1
        
        self.current_marker_array = marker_array
        return True

    def publish_markers(self, event=None):
        """
        发布当前存储的MarkerArray（由定时器调用）
        """
        if self.current_marker_array:
            # 更新每个marker的时间戳，这对于某些RViz配置很重要
            for marker in self.current_marker_array.markers:
                marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.current_marker_array)

    def _clear_all_markers(self):
        """
        发布一个特殊的Marker来清除RViz中之前所有的Markers
        """
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        rospy.loginfo("已清除旧的路径标记。")

    @staticmethod
    def _load_nav_points(file_path):
        """
        从带有注释的JSON文件中加载导航点
        """
        try:
            with open(file_path, 'r') as f:
                data = commentjson.load(f)
                return data
        except Exception as e:
            rospy.logerr(f"加载或解析JSON文件失败: {file_path}, 错误: {e}")
            return None

    @staticmethod
    def _create_path_marker(group_name, points, marker_id, color):
        """
        为单个路径组创建Marker消息
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = group_name  # 使用组名作为命名空间
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # 线宽
        marker.color = color

        for point_data in points:
            p = Point()
            p.x = point_data['position']['x']
            p.y = point_data['position']['y']
            p.z = point_data['position']['z']
            marker.points.append(p)
        return marker

    @staticmethod
    def _generate_random_color():
        """
        生成随机但鲜艳的颜色
        """
        return ColorRGBA(r=random.uniform(0.2, 1.0), g=random.uniform(0.2, 1.0), b=random.uniform(0.2, 1.0), a=1.0)

def main():
    rospy.init_node('path_visualizer_switcher')
    try:
        visualizer = PathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()