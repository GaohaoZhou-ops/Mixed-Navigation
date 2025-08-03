#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from math import sin, cos

class RobotSimulator:
    def __init__(self):
        """
        ROS节点初始化
        """
        # 初始化ROS节点
        rospy.init_node('simple_robot_simulator')
        rospy.loginfo("Starting Simple Robot Simulator Node...")

        # 从参数服务器获取配置，如果未设置则使用默认值
        self.odom_frame = rospy.get_param('~odom_frame_id', 'odom')
        self.base_frame = rospy.get_param('~base_frame_id', 'base_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)

        # 初始化机器人状态变量
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 初始化机器人速度变量
        self.v_x = 0.0
        self.v_theta = 0.0

        # 获取当前时间，用于计算时间差 (dt)
        self.last_time = rospy.Time.now()

        # 设置发布器和广播器
        self.pose_publisher = rospy.Publisher('/sim_loc', PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 设置订阅器
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)

        # 设置一个定时器，以固定的频率调用主更新循环
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.update_loop)
        
        rospy.loginfo("Simulator node initialized successfully.")

    def cmd_vel_callback(self, msg):
        """
        /cmd_vel 话题的回调函数，用于更新速度指令
        """
        self.v_x = msg.linear.x
        self.v_theta = msg.angular.z

    def initial_pose_callback(self, msg):
        """
        /initialpose 话题的回调函数，用于从Rviz重置机器人位姿
        """
        # 从消息中提取位姿和方向
        pose = msg.pose.pose
        orientation_q = pose.orientation
        
        # 将四元数转换为欧拉角，以获取偏航角 (yaw)
        _, _, yaw = tf_trans.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # 更新机器人内部状态
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = yaw
        
        # 重置速度
        self.v_x = 0.0
        self.v_theta = 0.0

        rospy.loginfo("Robot pose has been reset to [x=%.2f, y=%.2f, theta=%.2f] by initialpose.", 
                      self.x, self.y, self.theta)

    def update_loop(self, event):
        """
        主更新循环，由定时器触发
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # --- 1. 基于当前速度计算位姿变化 ---
        # 简单的欧拉积分模型
        delta_x = self.v_x * cos(self.theta) * dt
        delta_y = self.v_x * sin(self.theta) * dt
        delta_theta = self.v_theta * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # --- 2. 发布TF变换 (odom -> base_link) ---
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = tf_trans.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # --- 3. 发布PoseStamped消息到 /sim_loc 话题 ---
        p = PoseStamped()
        p.header.stamp = current_time
        p.header.frame_id = self.odom_frame

        p.pose.position.x = self.x
        p.pose.position.y = self.y
        p.pose.position.z = 0.0
        
        # 使用上面为TF计算的同一个四元数
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        self.pose_publisher.publish(p)

if __name__ == '__main__':
    try:
        RobotSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simulator node terminated.")