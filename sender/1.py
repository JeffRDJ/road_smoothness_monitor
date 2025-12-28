#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_matrix
from road_monitor.msg import RoadQuality  # 确保包名正确


class RoadSurfaceMonitor:
    def __init__(self):
        rospy.init_node('road_surface_monitor', anonymous=True)

        # 参数设置
        self.window_size = rospy.get_param('~window_size', 20)
        self.gravity = 9.80665
        self.accel_z_history = []

        self.threshold_smooth = 0.3
        self.threshold_rough = 0.8

        # 只需要一个发布者
        self.quality_pub = rospy.Publisher('/road_quality', RoadQuality, queue_size=10)

        # 订阅话题
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        rospy.loginfo("路面检测节点已启动 (集成消息版)")

    def imu_callback(self, msg):
        # 1. 姿态解算
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        if all(v == 0 for v in q):
            vertical_accel = msg.linear_acceleration.z - self.gravity
        else:
            rotation_matrix = quaternion_matrix(q)[:3, :3]
            accel_body = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])
            accel_world = np.dot(rotation_matrix, accel_body)
            vertical_accel = accel_world[2] - self.gravity

        # 2. 滑动窗口
        self.accel_z_history.append(vertical_accel)
        if len(self.accel_z_history) > self.window_size:
            self.accel_z_history.pop(0)

        # 3. 计算并发布
        if len(self.accel_z_history) == self.window_size:
            score = np.std(self.accel_z_history)

            # 状态分级
            status = "Smooth"
            if score > self.threshold_rough:
                status = "Very Rough / Bump"
            elif score > self.threshold_smooth:
                status = "Moderate"

            # 创建并填充自定义消息
            output = RoadQuality()
            # 关键：透传原始 IMU 数据的时间戳和坐标系 ID
            output.header.stamp = msg.header.stamp
            output.header.frame_id = msg.header.frame_id
            output.score = score
            output.status = status

            self.quality_pub.publish(output)


if __name__ == '__main__':
    try:
        monitor = RoadSurfaceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass