#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# reference paper: http://robots.stanford.edu/papers/thrun.stanley05.pdf
# longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서
# 1. subscriber, publisher 선언
# 2. 차량의 현재 위치와 경로 사이의 가장 가까운 점 찾기
# 3. 좌표 변환 행렬 생성 및 변환
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish


class stanley:
    def __init__(self):
        rospy.init_node("stanley", anonymous=False)

        # (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/lattice_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        self.ctrl_cmd_pub = rospy.Publisher(
            "ctrl_cmd_0", CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_status = False
        self.is_global_path = False
        self.forward_point = Point()
        self.current_position = Point()

        self.vehicle_length = 4.470
        self.stanley_gain = 0.5
        self.target_velocity = 60
        self.window_size = 10
        rate = rospy.Rate(30)

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity / 3.6, 0.15)
        while True:
            if self.is_global_path == True:
                self.velocitB_list = self.vel_planning.curvedBaseVelocity(
                    self.global_path, self.window_size
                )
                break
            else:
                print("--------------------------")
                print("Waiting global path data")
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = -1.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rospy.sleep(0.5)

        while not rospy.is_shutdown():

            if self.is_path == True and self.is_global_path == True and self.is_status == True:
                prev_time = time.time()

                self.current_waypoint = self.get_current_waypoint(
                    self.status_msg, self.global_path
                )
                self.target_velocity = self.velocitB_list[self.current_waypoint] * 3.6
                # TODO: 가까운 점이 확인 안 될 때 어떻게 할 것인지
                steering = self.calc_stanley()
                self.ctrl_cmd_msg.steering = steering

                output = self.pid.pid(
                    self.target_velocity, self.status_msg.velocity.x * 3.6
                )

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # (8) 제어입력 메세지 Publish
                print("--------------------------")
                print(
                    "current position: (",
                    round(self.current_position.x, 2),
                    ",",
                    round(self.current_position.y, 2),
                    ")",
                )
                # print("target velocity: ", round(self.target_velocity, 2))
                # print("current velocity: ", round(
                #     self.status_msg.velocity.x * 3.6, 2))
                # print("accel: ", round(self.ctrl_cmd_msg.accel, 2))
                # print("steering: ", round(steering, 2))
                current_time = time.time()
                # print("duration time: ", round(current_time - prev_time, 2))
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def status_callback(self, msg):  # Vehicle Status Subscriber
        self.is_status = True
        self.current_position.x = msg.position.x
        self.current_position.y = msg.position.y
        self.vehicle_yaw = np.deg2rad(msg.heading)
        self.status_msg = msg

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float("inf")
        current_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                current_waypoint = i
        return current_waypoint

    def calc_stanley(self):

        # (2) 차량의 현재 위치와 경로 사이의 가장 가까운 점 찾기
        min_dist = float("inf")
        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            dx = self.current_position.x - path_point.x
            dy = self.current_position.y - path_point.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                self.nearest_point = path_point
                self.nearest_point_num = num

        vehicle_position = self.current_position
        translation = [vehicle_position.x, vehicle_position.y]

        # (3) 좌표 변환 행렬 생성 및 변환
        num = 1
        while True:
            dx = self.path.poses[self.nearest_point_num + num].pose.position.x - self.nearest_point.x
            dy = self.path.poses[self.nearest_point_num + num].pose.position.y - self.nearest_point.y
            distance = sqrt(pow(dx, 2) + pow(dy, 2))
            num += 1
            if distance > 0.01:
                break
        path_yaw = atan2(dy, dx)

        trans_matrix = np.array(
            [
                [cos(path_yaw), -sin(path_yaw), translation[0]],
                [sin(path_yaw), cos(path_yaw), translation[1]],
                [0, 0, 1],
            ]
        )

        det_trans_matrix = np.linalg.inv(trans_matrix)
        global_path_point = [self.nearest_point.x, self.nearest_point.y, 1]
        local_path_point = det_trans_matrix.dot(global_path_point)

        # (4) Steering 각도 계산
        psi = path_yaw - self.vehicle_yaw
        steering = psi + atan2(
            self.stanley_gain * local_path_point[1], self.status_msg.velocity.x
        )
        print("nearest_point_global_path_point: (", round(
            global_path_point[0], 2), ",", round(global_path_point[1], 2), ")")
        print("nearest_point_local_path_point: (", round(local_path_point[0], 2),
              ",", round(local_path_point[1], 2), ")")
        print("path_yaw: ", round(path_yaw, 2))
        print("vehicle_yaw: ", round(self.vehicle_yaw, 2))
        print("psi: ", round(psi, 2))
        print("atan2: ", round(atan2(self.stanley_gain * local_path_point[1],
                                     self.status_msg.velocity.x), 2))
        print("steering: ", round(steering, 2))

        return steering


class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel

        # (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output


class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []

        for i in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(global_path.poses) - point_num):
            A_list = []
            B_list = []
            for box in range(-point_num, point_num):
                x = global_path.poses[i + box].pose.position.x
                y = global_path.poses[i + box].pose.position.y
                A_list.append([-2 * x, -2 * y, 1])
                B_list.append((-x * x) - (y * y))

            # (6) 도로의 곡률 계산, PPT p.390
            # Ax = B
            A_matrix = np.array(A_list)
            B_matrix = np.array(B_list)
            x_matrix = np.linalg.pinv(A_matrix).dot(B_matrix)

            a = x_matrix[0]
            b = x_matrix[1]
            c = x_matrix[2]
            r = sqrt(a * a + b * b - c)

            # (7) 곡률 기반 속도 계획
            # TODO: 회전할 때 r값이 얼마나 나오는지 출력해보기
            if r > 50:
                v_max = self.car_max_speed
            else:
                v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses) - 10):
            out_vel_plan.append(30)

        for i in range(len(global_path.poses) - 10, len(global_path.poses)):
            out_vel_plan.append(0)
        print("out_vel_plan: ", out_vel_plan)

        return out_vel_plan


if __name__ == "__main__":
    try:
        test_track = stanley()
    except rospy.ROSInterruptException:
        pass
