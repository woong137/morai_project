#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from lib.mgeo.class_defs import *
import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json

from math import cos, sin, sqrt, pow, atan2, pi
from geometry_msgs.msg import Point32, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)


class dijkstra_path_pub:
    def __init__(self):
        # Init Nodes
        rospy.init_node('dijkstra_path_pub', anonymous=True)
        self.global_path_pub = rospy.Publisher(
            '/global_path', Path, queue_size=1)

        map_link = 'lib/mgeo_data/R_KR_PR_SeongnamCityHall'

        waypoints = ["154S", "3E", "121S", "32E", "68S", "75S", "73S",
                     "85S", "82S", "78S", "124S", "122S", "8E", "111E", "203S"]

        load_path = os.path.normpath(os.path.join(
            current_path, map_link))

        # Get mgeo Data from json file
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set

        self.nodes = node_set.nodes
        self.links = link_set.lines

        self.global_planner = Dijkstra(self.nodes, self.links)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'

        self.intermediate_point_distance = rospy.get_param(
            'global_path/intermediate_point_distance', 1.0)
        rate = rospy.Rate(rospy.get_param('global_path/rate', 1))

        # self.intermediate_point_distance = 1.0
        # rate = rospy.Rate(1)

        self.global_path_is_calculated = False

        while not rospy.is_shutdown():
            # Making global path
            if not self.global_path_is_calculated:
                for i in range(len(waypoints)-1):

                    start_node, end_node = waypoints[i], waypoints[i+1]
                    result, path = self.global_planner.find_shortest_path(
                        start_node, end_node)

                    # 보정된 경로
                    corrected_path = self.correct_path(path["point_path"])

                    for waypoint in corrected_path:
                        path_x = waypoint[0]
                        path_y = waypoint[1]
                        read_pose = PoseStamped()
                        read_pose.pose.position.x = path_x
                        read_pose.pose.position.y = path_y
                        read_pose.pose.orientation.w = 1
                        self.global_path_msg.poses.append(read_pose)

            self.global_path_is_calculated = True

            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()

    def correct_path(self, original_path):
        corrected_path = [original_path[0]]  # 시작점 추가
        for i in range(1, len(original_path)):
            prev_point = original_path[i - 1]
            current_point = original_path[i]
            distance = sqrt(
                (current_point[0] - prev_point[0])**2 + (current_point[1] - prev_point[1])**2)
            if distance > self.intermediate_point_distance:  # 간격이 1m보다 큰 경우
                num_intermediate_points = int(
                    distance / self.intermediate_point_distance)  # 보정할 중간 점 개수
                delta_x = (current_point[0] - prev_point[0]) / \
                    (num_intermediate_points + 1)
                delta_y = (current_point[1] - prev_point[1]) / \
                    (num_intermediate_points + 1)
                for j in range(num_intermediate_points):
                    corrected_x = prev_point[0] + (j + 1) * delta_x
                    corrected_y = prev_point[1] + (j + 1) * delta_y
                    corrected_path.append([corrected_x, corrected_y])
            corrected_path.append(current_point)
        return corrected_path


class Dijkstra:
    def __init__(self, nodes, links):
        self.nodes = nodes
        self.links = links
        self.weight = self.get_weight_matrix()
        self.lane_change_link_idx = []

    def get_weight_matrix(self):
        weight = dict()
        for from_node_id, from_node in self.nodes.items():
            weight_from_this_node = dict()
            for to_node_id, to_node in self.nodes.items():
                weight_from_this_node[to_node_id] = float('inf')
            weight[from_node_id] = weight_from_this_node

        for from_node_id, from_node in self.nodes.items():
            weight[from_node_id][from_node_id] = 0

            for to_node in from_node.get_to_nodes():
                shortest_link, min_cost = self.find_shortest_link_leading_to_node(
                    from_node, to_node)
                weight[from_node_id][to_node.idx] = min_cost

        return weight

    def find_shortest_link_leading_to_node(self, from_node, to_node):
        to_links = []
        for link in from_node.get_to_links():
            if link.to_node is to_node:
                to_links.append(link)

        if len(to_links) == 0:
            raise BaseException(
                '[ERROR] Error @ Dijkstra.find_shortest_path : Internal data error. There is no link from node to node')

        shortest_link = None
        min_cost = float('inf')
        for link in to_links:
            if link.cost < min_cost:
                min_cost = link.cost
                shortest_link = link

        return shortest_link, min_cost

    def find_nearest_node_idx(self, distance, s):
        idx_list = list(self.nodes.keys())
        min_value = float('inf')
        min_idx = idx_list[-1]

        for idx in idx_list:
            if distance[idx] < min_value and s[idx] == False:
                min_value = distance[idx]
                min_idx = idx
        return min_idx

    def find_shortest_path(self, start_node_idx, end_node_idx):
        s = dict()
        from_node = dict()
        for node_id in self.nodes.keys():
            s[node_id] = False
            from_node[node_id] = start_node_idx

        s[start_node_idx] = True
        distance = copy.deepcopy(self.weight[start_node_idx])

        for i in range(len(self.nodes.keys()) - 1):
            selected_node_idx = self.find_nearest_node_idx(distance, s)
            s[selected_node_idx] = True
            for j, to_node_idx in enumerate(self.nodes.keys()):
                if s[to_node_idx] == False:
                    distance_candidate = distance[selected_node_idx] + \
                        self.weight[selected_node_idx][to_node_idx]
                    if distance_candidate < distance[to_node_idx]:
                        distance[to_node_idx] = distance_candidate
                        from_node[to_node_idx] = selected_node_idx

        tracking_idx = end_node_idx
        node_path = [end_node_idx]

        while start_node_idx != tracking_idx:
            tracking_idx = from_node[tracking_idx]
            node_path.append(tracking_idx)

        node_path.reverse()

        link_path = []
        for i in range(len(node_path) - 1):
            from_node_idx = node_path[i]
            to_node_idx = node_path[i + 1]

            from_node = self.nodes[from_node_idx]
            to_node = self.nodes[to_node_idx]

            shortest_link, min_cost = self.find_shortest_link_leading_to_node(
                from_node, to_node)
            link_path.append(shortest_link.idx)

        if len(link_path) == 0:
            return False, {'node_path': node_path, 'link_path': link_path, 'point_path': []}

        point_path = []
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])

        return True, {'node_path': node_path, 'link_path': link_path, 'point_path': point_path}


if __name__ == '__main__':

    dijkstra_path_pub = dijkstra_path_pub()
