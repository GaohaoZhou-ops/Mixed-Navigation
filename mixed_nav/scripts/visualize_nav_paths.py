#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import commentjson
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def generate_random_color():
    """
    生成随机但鲜艳的颜色
    """
    import random
    return ColorRGBA(
        r=random.uniform(0.2, 1.0),
        g=random.uniform(0.2, 1.0),
        b=random.uniform(0.2, 1.0),
        a=1.0
    )

def load_nav_points(file_path):
    """
    从带有注释的JSON文件中加载导航点
    """
    try:
        with open(file_path, 'r') as f:
            data = commentjson.load(f)
            return data
    except Exception as e:
        rospy.logerr("Failed to load or parse JSON file: %s", e)
        return None

def create_path_marker(group_name, points, marker_id, color):
    """
    为单个路径组创建Marker消息
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = group_name
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.05  # Line width

    marker.color = color

    for point_data in points:
        p = Point()
        p.x = point_data['position']['x']
        p.y = point_data['position']['y']
        p.z = point_data['position']['z']
        marker.points.append(p)

    return marker

def main():
    rospy.init_node('nav_path_visualizer')
    
    # 从参数服务器获取JSON文件路径，如果未设置则使用默认值
    json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')

    nav_point_groups = load_nav_points(json_file_path)

    if not nav_point_groups:
        return

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    
    marker_array = MarkerArray()
    marker_id_counter = 0

    for group_name, points in nav_point_groups.items():
        color = generate_random_color()
        path_marker = create_path_marker(group_name, points, marker_id_counter, color)
        marker_array.markers.append(path_marker)
        marker_id_counter += 1

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        for marker in marker_array.markers:
            marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass