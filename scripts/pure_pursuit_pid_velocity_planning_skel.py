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

# advanced_purepursuit 은 차량의 차량의 종 횡 방향 제어 예제입니다.
# Purpusuit 알고리즘의 Look Ahead Distance 값을 속도에 비례하여 가변 값으로 만들어 횡 방향 주행 성능을 올립니다.
# 횡방향 제어 입력은 주행할 Local Path (지역경로) 와 차량의 위치 정보 Ego_topic를 받아 차량을 제어 합니다.
# 종방향 제어 입력은 목표 속도를 지정 한뒤 목표 속도에 도달하기 위한 Throttle control 을 합니다.
# 종방향 제어 입력은 longlCmdType 1(Throttle control) 이용합니다.

# 노드 실행 순서
# 1. subscriber, publisher 선언
# 2. 속도 비례 Look Ahead Distance 값 설정
# 3. 좌표 변환 행렬 생성
# 4. Steering 각도 계산
# 5. PID 제어 생성
# 6. 도로의 곡률 계산
# 7. 곡률 기반 속도 계획
# 8. 제어입력 메세지 Publish


class pure_pursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=False)

        # (1) subscriber, publisher 선언
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher(
            "ctrl_cmd_0", CtrlCmd, queue_size=1)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 4.470
        self.lfd = 3.0
        self.min_lfd = 3
        self.max_lfd = 15
        self.lfd_gain = 0.78
        self.target_velocity = 60
        self.window_size = 200

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

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_status == True and self.is_global_path == True:
                prev_time = time.time()

                self.current_waypoint = self.get_current_waypoint(
                    self.status_msg, self.global_path
                )
                self.target_velocity = self.velocitB_list[self.current_waypoint] * 3.6

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

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
                    round(self.current_postion.x, 2),
                    ",",
                    round(self.current_postion.y, 2),
                    ")",
                )
                print("target velocity: ", self.target_velocity)
                print("current velocity: ", self.status_msg.velocity.x * 3.6)
                print("accel: ", self.ctrl_cmd_msg.accel)
                print("steering: ", steering)
                print("lfd: ", self.lfd)
                current_time = time.time()
                print("duration time: ", current_time - prev_time)
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

    def calc_pure_pursuit(self):

        # (2) 속도 비례 Look Ahead Distance 값 설정
        self.lfd = (self.status_msg.velocity.x) * self.lfd_gain
        self.lfd = np.clip(self.lfd, self.min_lfd, self.max_lfd)

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        # (3) 좌표 변환 행렬 생성
        trans_matrix = np.array(
            [
                [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                [0, 0, 1],
            ]
        )

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0:
                dis = sqrt(
                    pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break
                # print("local_path_point: ", local_path_point)
                # print("dis: ", dis)

        # (4) Steering 각도 계산
        theta = atan2(local_path_point[1], local_path_point[0])
        steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)

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
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass
