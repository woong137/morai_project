#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import EgoVehicleStatus
import tf


class path_pub:

    def __init__(self):
        rospy.init_node("local_path_publisher", anonymous=False)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.Subscriber("/global_path", Path, self.global_Path_callback)

        self.local_path_pub = rospy.Publisher(
            "/local_path", Path, queue_size=1)

        # 초기화
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "/map"

        self.is_status = False
        self.x = 0
        self.y = 0
        self.current_waypoint = -1

        self.local_path_size = rospy.get_param('~local_path/size', 30)
        self.threshold = rospy.get_param('~local_path/threshold', 10)
        rate = rospy.Rate(rospy.get_param('~local_path/rate', 30))

        # rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            if self.is_status == True:
                local_path_msg = Path()
                local_path_msg.header.frame_id = "/map"

                x = self.x
                y = self.y

                min_dis = float("inf")

                for i, waypoint in enumerate(self.global_path_msg.poses):
                    # 과거 웨이포인트와 인덱스 차이가 임계값 이내인 경우
                    if 0 <= i - self.current_waypoint <= self.threshold:

                        distance = sqrt(
                            pow(x - waypoint.pose.position.x, 2)
                            + pow(y - waypoint.pose.position.y, 2)
                        )
                        if distance < min_dis:
                            min_dis = distance
                            self.current_waypoint = i

                if self.current_waypoint != -1:
                    if self.current_waypoint + self.local_path_size < len(
                        self.global_path_msg.poses
                    ):
                        for num in range(
                            self.current_waypoint, self.current_waypoint + self.local_path_size
                        ):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.global_path_msg.poses[
                                num
                            ].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[
                                num
                            ].pose.position.y
                            tmp_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(tmp_pose)

                    else:
                        for num in range(
                            self.current_waypoint, len(
                                self.global_path_msg.poses)
                        ):
                            tmp_pose = PoseStamped()
                            tmp_pose.pose.position.x = self.global_path_msg.poses[
                                num
                            ].pose.position.x
                            tmp_pose.pose.position.y = self.global_path_msg.poses[
                                num
                            ].pose.position.y
                            tmp_pose.pose.orientation.w = 1
                            local_path_msg.poses.append(tmp_pose)

                self.local_path_pub.publish(local_path_msg)
            rate.sleep()

    def status_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.is_status = True

    def global_Path_callback(self, msg):
        self.global_path_msg = msg


if __name__ == "__main__":
    try:
        test_track = path_pub()
    except rospy.ROSInterruptException:
        pass
