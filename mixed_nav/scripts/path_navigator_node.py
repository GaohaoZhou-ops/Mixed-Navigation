#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import commentjson
import tf
import math

from geometry_msgs.msg import PoseStamped
# 导入刚刚编译生成的Action消息文件
from mixed_nav.msg import PathNavigationAction, PathNavigationGoal, PathNavigationResult, PathNavigationFeedback

def load_nav_points(file_path):
    """从带有注释的JSON文件中加载导航点"""
    try:
        with open(file_path, 'r') as f:
            data = commentjson.load(f)
            return data
    except Exception as e:
        rospy.logerr("Failed to load or parse JSON file: %s", e)
        return None

class PathNavigator:
    def __init__(self):
        rospy.init_node('path_navigator_node')

        # 从参数服务器获取JSON文件路径
        json_file_path = rospy.get_param('~json_file_path', 'nav_points.json')
        self.nav_point_groups = load_nav_points(json_file_path)

        if not self.nav_point_groups:
            rospy.logerr("Failed to load navigation points. Shutting down.")
            return

        # TF监听器，用于获取机器人位置
        self.tf_listener = tf.TransformListener()
        # 目标点发布者
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        # 创建Action服务器
        self._action_server = actionlib.SimpleActionServer(
            '/track_points', 
            PathNavigationAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        self._action_server.start()

        rospy.loginfo("Path Navigation Action Server is ready.")

    def execute_cb(self, goal):
        rospy.loginfo("Received navigation goal for path group: '%s' with dead zone radius: %.2f", 
                      goal.path_group_name, goal.dead_zone_radius)

        # 1. 验证路径组是否存在
        if goal.path_group_name not in self.nav_point_groups:
            rospy.logerr("Path group '%s' not found in JSON file.", goal.path_group_name)
            self._action_server.set_aborted(result=PathNavigationResult(success=False, message="Path group not found."))
            return

        waypoints = self.nav_point_groups[goal.path_group_name]
        rospy.loginfo("Executing path with %d waypoints.", len(waypoints))

        # 2. 循环遍历路径点
        for i, waypoint_data in enumerate(waypoints):
            # 检查Action是否被客户端取消
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Navigation preempted by client.")
                self._action_server.set_preempted()
                return

            # 创建并发布目标点
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map" # 假设导航点在map坐标系下
            
            goal_pose.pose.position.x = waypoint_data['position']['x']
            goal_pose.pose.position.y = waypoint_data['position']['y']
            goal_pose.pose.position.z = waypoint_data['position']['z']
            goal_pose.pose.orientation.x = waypoint_data['orientation']['x']
            goal_pose.pose.orientation.y = waypoint_data['orientation']['y']
            goal_pose.pose.orientation.z = waypoint_data['orientation']['z']
            goal_pose.pose.orientation.w = waypoint_data['orientation']['w']
            
            self.goal_pub.publish(goal_pose)
            
            # 发布反馈
            feedback = PathNavigationFeedback()
            feedback.current_waypoint_info = "Navigating to waypoint %d of %d in group '%s'." % (i + 1, len(waypoints), goal.path_group_name)
            self._action_server.publish_feedback(feedback)
            rospy.loginfo(feedback.current_waypoint_info)

            # 3. 等待机器人到达死区
            self.wait_for_arrival(goal_pose, goal.dead_zone_radius)
            
            # 如果在等待时被抢占，则退出
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Navigation preempted by client while waiting for arrival.")
                self._action_server.set_preempted()
                return
            
            rospy.loginfo("Waypoint %d reached.", i + 1)

        # 4. 所有点都成功到达
        result = PathNavigationResult(success=True, message="Navigation completed successfully.")
        self._action_server.set_succeeded(result)
        rospy.loginfo("Path group '%s' completed successfully.", goal.path_group_name)

    def wait_for_arrival(self, target_pose, radius):
        """等待机器人到达目标点指定的半径内"""
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                break
            try:
                # 获取机器人基座(base_link)在地图(map)坐标系中的位置
                (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
                
                # 计算2D距离
                dx = trans[0] - target_pose.pose.position.x
                dy = trans[1] - target_pose.pose.position.y
                distance = math.sqrt(dx*dx + dy*dy)

                if distance < radius:
                    rospy.loginfo("Distance to goal (%.2f m) is within dead zone (%.2f m). Arrived.", distance, radius)
                    break
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF Exception: %s. Retrying...", e)
                continue
            
            rate.sleep()

if __name__ == '__main__':
    from mixed_nav.msg import PathNavigationAction

    try:
        PathNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass