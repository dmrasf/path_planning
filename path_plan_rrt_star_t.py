import numpy as np
import matplotlib.pyplot as plt
from my_map import Map
import json
import random
import math


class PathPlanningRRTStarV():
    """RRT*V算法"""

    def __init__(self, my_map: Map):
        self.__my_map = my_map
        self.__map = my_map.get_map()
        self.__visual_points = self.__my_map.get_visual_points()
        self.__visual_graph = self.__my_map.get_visual_graph()
        self.__start = 0
        self.__end = len(self.__visual_points)-1

    def __get_random_point(self):
        while True:
            i = random.randint(1, len(self.__visual_points)-2)
            if i not in self.__close_points_set:
                return i

    def __find_near_point_from_tree_for_star(self):
        distance = float('inf')
        near = -1
        for node in self.__close_points_set:
            if self.__visual_graph[node, self.__x_rand] <= 0:
                continue
            tmp = self.__visual_graph[node, self.__x_rand] + \
                self.__calculate_path_dis(node)
            if tmp < distance:
                distance = tmp
                near = node
        return near

    def __add_new_branch(self):
        self.__tree[self.__x_new] = self.__x_near
        self.__close_points_set.add(self.__x_new)

    def __change_parent(self):
        dis = self.__calculate_path_dis(self.__x_new)
        for node in self.__close_points_set:
            if node == self.__x_new or node == self.__tree[self.__x_new]:
                continue
            if self.__visual_graph[self.__x_new, node] <= 0:
                continue
            tmp = self.__visual_graph[node, self.__x_new]
            if dis + tmp < self.__calculate_path_dis(node):
                self.__tree[node] = self.__x_new
                self.__point_to_start_distance[node] = dis + tmp

    def __is_arrived(self):
        if self.__visual_graph[self.__x_new, self.__end] > 0:
            return True
        return False

    def __calculate_path_dis(self, point):
        if point not in self.__point_to_start_distance:
            dis = 0
            path = [point]
            while path[-1] != self.__start:
                path.append(self.__tree[path[-1]])
                dis = dis + self.__visual_graph[path[-1], path[-2]]
            self.__point_to_start_distance[point] = dis
        return self.__point_to_start_distance[point]

    def __parse_path(self):
        path = [self.__end]
        self.__path = [self.__visual_points[self.__end]]
        while path[-1] != self.__start:
            path.append(self.__tree[path[-1]])
            self.__path.append(self.__visual_points[path[-1]])

    def start_planing(self):
        """开始规划"""
        self.__tree = dict()
        self.__close_points_set = set()
        self.__close_points_set.add(0)
        self.__point_to_start_distance = dict()

        for i in range(100000):
            self.__x_rand = self.__get_random_point()
            self.__x_near = self.__find_near_point_from_tree_for_star()
            if self.__x_near == -1:
                continue
            self.__x_new = self.__x_rand
            self.__add_new_branch()
            self.__change_parent()
            if self.__is_arrived():
                print(i)
                self.__tree[self.__end] = self.__x_new
                self.__parse_path()
                break
        else:
            print('未找到')
            return None
        return self.__path


my_map = Map('./map/map_data_4.json', 'json')
plan_rrt_star = PathPlanningRRTStarV(my_map)
print('========================================')
points_rrt_star = plan_rrt_star.start_planing()
path_rrt_star = my_map.calculate_path_distance(points_rrt_star)
print('路径总长度', path_rrt_star)
print('========================================')
my_map.show_map('RRT*', points=points_rrt_star)
