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
# 2. 앞바퀴 중심점과 경로 사이의 가장 가까운 점 찾기
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
        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        rospy.wait_for_service('/Service_MoraiEventCmd')

        self.ctrl_cmd_pub = rospy.Publisher(
            "ctrl_cmd_0", CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_status = False
        self.is_global_path = False
        self.switcher = "driving"
        self.current_position = Point()
        self.end_position = Point(166.0, -104.2, 0.0)
        self.stop_initiation_distance = 80
        self.switch_stop_initiation_tolerance = 0.5
        self.prev_steering = 0.0

        self.wheel_base = 2.7
        self.stanley_gain = 1.0
        self.target_velocity = 50  # km/h
        self.window_size = 50
        rate = rospy.Rate(50)

        self.vel_pid = pidControl(0.3, 0.0, 0.03)
        self.pos_pid = pidControl(0.5, 0.0, 0.0)

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
                self.ctrl_cmd_msg.brake = 1.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.is_path == True and self.is_global_path == True and self.is_status == True:
                print("switcher: ", self.switcher)
                front_wheel_position = Point()
                front_wheel_position.x = (
                    self.current_position.x +
                    self.wheel_base * cos(self.vehicle_yaw)
                )
                front_wheel_position.y = (
                    self.current_position.y +
                    self.wheel_base * sin(self.vehicle_yaw)
                )
                self.current_waypoint = self.get_current_waypoint(
                    self.status_msg, self.global_path
                )
                self.target_velocity = self.velocitB_list[self.current_waypoint] * 3.6
                steering = self.calc_stanley(front_wheel_position)
                self.ctrl_cmd_msg.steering = steering

                if self.switcher == "driving":
                    acc_input = self.vel_pid.pid(
                        self.target_velocity, self.status_msg.velocity.x * 3.6
                    )

                    if acc_input > 0.0:
                        self.ctrl_cmd_msg.accel = acc_input
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -acc_input

                    # (8) 제어입력 메세지 Publish
                    # print("--------------------------")
                    print(
                        "current position: (",
                        round(self.current_position.x, 2),
                        ",",
                        round(self.current_position.y, 2),
                        ")",
                    )
                    print("target velocity: ", round(self.target_velocity, 2))
                    print("current velocity: ", round(
                        self.status_msg.velocity.x * 3.6, 2))
                    # print("accel: ", round(self.ctrl_cmd_msg.accel, 2))
                    print("steering: ", round(steering, 2))
                    dis = self.stop_initiation_distance
                    tol = self.switch_stop_initiation_tolerance
                    if self.end_position.x - dis - tol < self.current_position.x < self.end_position.x - dis + tol \
                            and self.end_position.y - tol < self.current_position.y < self.end_position.y + tol:
                        self.switcher = "stop"

                elif self.switcher == "stop":
                    self.ctrl_cmd_msg.longlCmdType = 2
                    vel_input = self.pos_pid.pid(
                        self.end_position.x, self.current_position.x)
                    # vel_input vel_max로 제한
                    if vel_input > self.target_velocity:
                        vel_input = self.target_velocity
                    self.ctrl_cmd_msg.velocity = vel_input

                    arrived_tol = 0.3
                    distance = sqrt(pow(self.current_position.x - self.end_position.x, 2) +
                                    pow(self.current_position.y - self.end_position.y, 2))
                    if distance < arrived_tol:
                        self.switcher = "arrived"

                elif self.switcher == "arrived":
                    self.ctrl_cmd_msg.longlCmdType = 2
                    self.ctrl_cmd_msg.velocity = 0.0
                    print("##############")
                    print("#Goal Reached#")
                    print("##############")

                else:
                    print("switcher error")
                    break

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            print("--------------------------")

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

    def distance_with_point_and_line(self, point1, point2, ego_point):
        # 분자 계산: 점 P에서 직선 AB까지의 거리 공식
        numerator = (point2.x - point1.x) * (point1.y - ego_point.y) - \
            (point1.x - ego_point.x) * (point2.y - point1.y)
        # 분모 계산: 직선 AB의 길이
        denominator = np.sqrt((point2.x - point1.x) ** 2 +
                              (point2.y - point1.y) ** 2)
        # 거리 계산
        distance = numerator / denominator

        # 외적을 사용하여 P가 직선 AB의 왼쪽에 있는지 오른쪽에 있는지 판단
        # cross_product = (point2.x - point1.x) * (ego_point.y - point1.y) - \
        #     (ego_point.x - point1.x) * (point2.y - point1.y)

        # # 외적의 부호에 따라 거리의 부호 결정
        # if cross_product > 0:
        #     # P가 AB의 왼쪽에 있으면 양수
        #     return distance
        # else:
        #     # P가 AB의 오른쪽에 있으면 음수
        return distance

    def calc_stanley(self, front_wheel_position):
        # (2) 차량의 앞바퀴 중심점과 경로 사이의 가장 가까운 점 찾기
        min_dist = float("inf")
        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position
            dx = front_wheel_position.x - path_point.x
            dy = front_wheel_position.y - path_point.y
            dist = sqrt(pow(dx, 2) + pow(dy, 2))
            if min_dist > dist:
                min_dist = dist
                nearest_point = path_point
                nearest_point_num = num

        translation = [front_wheel_position.x, front_wheel_position.y]

        num = 1
        while True:
            # print("nearest_point: (", round(nearest_point.x, 2), ",", round(
            #     nearest_point.y, 2), ")")
            if nearest_point_num + num >= len(self.path.poses):
                return self.prev_steering
            else:
                dx = self.path.poses[nearest_point_num +
                                     num].pose.position.x - nearest_point.x
                dy = self.path.poses[nearest_point_num +
                                     num].pose.position.y - nearest_point.y
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
        global_path_point = [nearest_point.x, nearest_point.y, 1]
        local_path_point = det_trans_matrix.dot(global_path_point)
        # distance_with_point_and_line = self.distance_with_point_and_line(
        #     self.path.poses[nearest_point_num].pose.position,
        #     self.path.poses[nearest_point_num + num].pose.position,
        #     front_wheel_position
        # )
        # print("distance_with_point_and_line: ", distance_with_point_and_line)

        # (4) Steering 각도 계산
        psi = path_yaw - self.vehicle_yaw
        psi = atan2(sin(psi), cos(psi))
        steering = psi + atan2(
            self.stanley_gain * local_path_point[1], self.status_msg.velocity.x
        )
        print("path_yaw: ", path_yaw)
        print("vehicle_yaw: ", self.vehicle_yaw)
        print("psi: ", psi)
        print("local_path_point[1]: ", local_path_point[1])
        self.prev_steering = steering

        return steering


class pidControl:
    def __init__(self, p_gain=0.3, i_gain=0.00, d_gain=0.03):
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self, target, current):
        error = target - current

        # (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        return output


class velocityPlanning:
    def __init__(self, car_max_speed, road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction

    def curvedBaseVelocity(self, global_path, point_num):
        out_vel_plan = []
        # TODO: point_num보다 거리를 기준으로 하는 것이 더 좋을 수도 있음
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
            v_max = sqrt(r * 9.8 * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses) - point_num, len(global_path.poses)):
            out_vel_plan.append(v_max)

        print("out_vel_plan: ", out_vel_plan)

        return out_vel_plan


if __name__ == "__main__":
    try:
        test_track = stanley()
    except rospy.ROSInterruptException:
        pass
