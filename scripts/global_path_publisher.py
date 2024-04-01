#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


def create_straight_line_path_with_intervals(start, end, interval=0.1):
    # 경로 메시지 초기화
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    # 시작점과 끝점 사이의 거리 계산
    distance = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
    # 총 포즈 개수 계산 (간격 포함)
    steps = int(distance / interval)

    for step in range(steps + 1):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        # 선형 보간을 사용하여 각 포즈의 위치 계산
        pose.pose.position.x = start[0] + (end[0] - start[0]) * (step / steps)
        pose.pose.position.y = start[1] + (end[1] - start[1]) * (step / steps)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return path


def publish_path():
    rospy.init_node("global_path_publisher", anonymous=False)
    path_pub = rospy.Publisher("/global_path", Path, queue_size=10)
    rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():
        straight_path = create_straight_line_path_with_intervals(
            (-145, -104), (150, -104), 0.1
        )
        path_pub.publish(straight_path)
        rate.sleep()


if __name__ == "__main__":
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
