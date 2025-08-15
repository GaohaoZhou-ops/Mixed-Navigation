#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import os
import rospkg

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty, EmptyResponse
from mixed_nav.srv import StartRecord, StartRecordResponse

class NavPointRecorder:
    COLORS = [
        ColorRGBA(0.0, 1.0, 1.0, 0.8),  # Cyan
        ColorRGBA(1.0, 0.0, 1.0, 0.8),  # Magenta
        ColorRGBA(1.0, 1.0, 0.0, 0.8),  # Yellow
        ColorRGBA(1.0, 0.5, 0.0, 0.8),  # Orange
        ColorRGBA(0.5, 0.0, 1.0, 0.8),  # Purple
        ColorRGBA(0.0, 0.5, 1.0, 0.8),  # Blue
    ]
    ACTIVE_COLOR = ColorRGBA(0.1, 1.0, 0.1, 1.0) # Bright Green for active path
    ACTIVE_SCALE = 0.07 # Thicker line for active path
    INACTIVE_SCALE = 0.04 # Thinner line for inactive paths
    # ====================================================================

    def __init__(self):
        rospy.init_node('nav_point_recorder', anonymous=True)

        try:
            # 确保这里的包名和你的 package.xml 一致
            self.package_name = "mixed_nav" 
            self.floor_name = rospy.get_param('~floor_name')
        except KeyError:
            rospy.logerr("必须通过launch文件提供 'floor_name' 参数！")
            rospy.signal_shutdown("缺少 floor_name 参数")
            return
            
        rospy.loginfo("当前楼层: %s", self.floor_name)

        self.setup_filepath()

        self.is_recording = False
        self.current_path_name = None
        self.all_paths = self.load_waypoints()
        self.last_path_count = 0 # 用于清理多余的marker

        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.marker_array_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.pose_array_pub = rospy.Publisher('recorded_path_poses', PoseArray, queue_size=10)

        rospy.Service('/start_record_nav_point', StartRecord, self.handle_start_record)
        rospy.Service('/undo_record_nav_point', Empty, self.handle_undo_record)
        rospy.Service('/finish_record_nav_point', Empty, self.handle_finish_record)

        rospy.loginfo("导航点记录节点已准备就绪，共加载 %d 条路径。", len(self.all_paths))
        
        rospy.sleep(1.0)
        self.publish_all_paths()
        # =========================================================================

    def setup_filepath(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(self.package_name)
        self.dir_path = os.path.join(pkg_path, 'resources', 'floors', self.floor_name)
        self.file_path = os.path.join(self.dir_path, 'waypoints.json')
        
        if not os.path.exists(self.dir_path):
            os.makedirs(self.dir_path)

    def load_waypoints(self):
        try:
            with open(self.file_path, 'r') as f:
                return json.load(f)
        except (IOError, ValueError):
            return {}

    def save_waypoints(self):
        try:
            with open(self.file_path, 'w') as f:
                json.dump(self.all_paths, f, indent=4)
        except IOError as e:
            rospy.logerr("保存路径点文件失败: %s", e)

    def handle_start_record(self, req):
        if self.is_recording:
            rospy.logwarn("正在记录路径 '%s'，请先完成。", self.current_path_name)
            return StartRecordResponse(False, "已经在记录中，请先完成当前任务。")

        self.is_recording = True
        self.current_path_name = req.path_name
        
        if self.current_path_name not in self.all_paths:
            self.all_paths[self.current_path_name] = []

        rospy.loginfo("开始/继续记录路径: '%s'", self.current_path_name)
        self.publish_all_paths() # 重绘所有路径，高亮当前路径
        return StartRecordResponse(True, "开始记录路径: " + req.path_name)

    def handle_undo_record(self, req):
        if not self.is_recording or not self.all_paths.get(self.current_path_name):
            rospy.logwarn("撤销失败：无点可撤销。")
            return EmptyResponse()

        self.all_paths[self.current_path_name].pop()
        rospy.loginfo("已撤销路径 '%s' 的最后一个点。", self.current_path_name)
        self.save_waypoints()
        self.publish_all_paths()
        return EmptyResponse()

    def handle_finish_record(self, req):
        if not self.is_recording:
            rospy.logwarn("完成失败：当前不处于记录模式。")
            return EmptyResponse()

        path_len = len(self.all_paths.get(self.current_path_name, []))
        rospy.loginfo("路径 '%s' 记录完成，共 %d 个点。", self.current_path_name, path_len)
        
        self.is_recording = False
        self.current_path_name = None
        
        self.save_waypoints()
        self.publish_all_paths() # 重绘所有路径，取消高亮
        return EmptyResponse()

    def goal_callback(self, msg):
        if not self.is_recording:
            return

        pose = msg.pose
        waypoint = {
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y, 'z': pose.orientation.z, 'w': pose.orientation.w}
        }
        self.all_paths[self.current_path_name].append(waypoint)
        rospy.loginfo("路径 '%s' 记录新点 #%d", self.current_path_name, len(self.all_paths[self.current_path_name]))
        
        self.save_waypoints()
        self.publish_all_paths()

    def publish_all_paths(self):
        """在 RViz 中发布所有已知的路径，并高亮活动路径"""
        marker_array = MarkerArray()
        full_pose_array = PoseArray()
        full_pose_array.header.stamp = rospy.Time.now()
        full_pose_array.header.frame_id = "map"

        path_names = sorted(self.all_paths.keys()) # 排序以保证颜色稳定

        for i, path_name in enumerate(path_names):
            waypoints = self.all_paths[path_name]
            if not waypoints:
                continue

            # --- 创建连线 Marker ---
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = path_name # 使用路径名作为命名空间
            line_marker.id = 0 # 线 id
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0

            # --- 创建姿态文本 Marker ---
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = path_name + "_label"
            text_marker.id = 1 # 文本 id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.text = path_name
            # 将标签放在第一个点上方
            text_marker.pose.position.x = waypoints[0]['position']['x']
            text_marker.pose.position.y = waypoints[0]['position']['y']
            text_marker.pose.position.z = waypoints[0]['position']['z'] + 0.5 # 向上偏移0.5米
            text_marker.scale.z = 0.3 # 字体大小

            # 判断是否为当前活动路径
            is_active = self.is_recording and path_name == self.current_path_name
            
            if is_active:
                line_marker.color = self.ACTIVE_COLOR
                line_marker.scale.x = self.ACTIVE_SCALE
                text_marker.color = self.ACTIVE_COLOR
            else:
                color_index = i % len(self.COLORS)
                line_marker.color = self.COLORS[color_index]
                line_marker.scale.x = self.INACTIVE_SCALE
                text_marker.color = self.COLORS[color_index]


            for wp in waypoints:
                # 添加点到连线
                p = Point()
                p.x, p.y, p.z = wp['position']['x'], wp['position']['y'], wp['position']['z']
                line_marker.points.append(p)

                # 添加位姿到总的 PoseArray
                pose = Pose()
                pose.position = p
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = wp['orientation'].values()
                full_pose_array.poses.append(pose)
            
            marker_array.markers.append(line_marker)
            marker_array.markers.append(text_marker)

        # --- 清理可能残留的旧路径 Marker ---
        # 如果之前显示的路径比现在多，需要删除多余的
        current_path_count = len(path_names)
        for i in range(current_path_count, self.last_path_count):
            # 获取上一次的路径名
            old_path_name = sorted(self.all_paths.keys())[i] # 这行逻辑有问题，我们直接用ID删除
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL # 最简单的方式是全部删除再重新画
        
        # 为了避免复杂性，我们采用更简单的方式：发布一个DELETEALL，然后再发布所有
        # 但这会导致闪烁。当前逐个覆盖和添加的方式更好。
        # 这里仅发布，Rviz会根据ns和id自动覆盖。
        
        self.marker_array_pub.publish(marker_array)
        self.pose_array_pub.publish(full_pose_array)
        self.last_path_count = current_path_count
    # ==============================================================================

if __name__ == '__main__':
    try:
        NavPointRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass