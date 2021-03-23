import numpy as np
import matplotlib.pyplot as plt
from my_map import Map
import json
import random
import math


class PathPlanningRRTStar():
    """RRT*算法"""

    def __init__(self, my_map: Map):
        self.__my_map = my_map
        self.__map = my_map.get_map()
        self.__start = tuple(my_map.real_to_grid(my_map.get_start_point()))
        self.__end = tuple(my_map.real_to_grid(my_map.get_end_point()))
        self.__branch_len = 30

    def __get_random_point(self):
        while True:
            x = random.randint(0, len(self.__map[:, 0])-1)
            y = random.randint(0, len(self.__map[0, :])-1)
            if (x, y) not in self.__ed and self.__map[x, y] == 255:
                if self.__my_map.is_visible(self.__x_current, [x, y]):
                    return (x, y)

    def __calculate_dis(self, p1, p2):
        return pow(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2), 0.5)

    def __find_near_point_from_tree_for_star(self):
        distance = float('inf')
        r = 1000
        near = ()
        for node in self.__ed:
            tmp = self.__calculate_dis(node, self.__x_rand)
            if tmp >= r:
                continue
            tmp = tmp + self.__calculate_path_dis(node)
            if tmp < distance and self.__my_map.is_visible(node, self.__x_rand):
                distance = tmp
                near = node
        return near

    def __find_near_point_from_tree(self):
        distance = float('inf')
        near = ()
        for node in self.__ed:
            tmp = self.__calculate_dis(node, self.__x_rand)
            if tmp < distance and self.__my_map.is_visible(node, self.__x_rand):
                distance = tmp
                near = node
        return near

    def __calculate_new_node(self):
        dis = self.__calculate_dis(self.__x_rand, self.__x_near)
        x_new = (self.__x_rand[0], self.__x_rand[1])
        if dis > self.__branch_len:
            x = self.__x_near[0] + (self.__x_rand[0] -
                                    self.__x_near[0])*self.__branch_len/dis
            y = self.__x_near[1] + (self.__x_rand[1] -
                                    self.__x_near[1])*self.__branch_len/dis
            x_new = (math.floor(x), math.floor(y))
        return x_new

    def __add_new_branch(self):
        self.__x_current = [self.__x_new[0], self.__x_new[1]]
        self.__t[self.__x_new] = self.__x_near
        self.__ed.add(self.__x_new)

    def __change_parent(self):
        r = 300
        dis = self.__calculate_path_dis(self.__x_new)
        for node in self.__ed:
            if node == self.__x_new or node == self.__t[self.__x_new]:
                continue
            tmp = self.__calculate_dis(node, self.__x_new)
            if tmp >= r:
                continue
            if not self.__my_map.is_visible(self.__x_new, node):
                continue
            if dis + tmp < self.__calculate_path_dis(node):
                self.__t[node] = self.__x_new
                self.__dis[node] = dis + tmp

    def __is_arrived(self):
        if self.__my_map.is_visible(self.__x_new, self.__end) and \
                self.__calculate_dis(self.__x_new, self.__end) <= self.__branch_len:
            return True
        return False

    def __calculate_path_dis(self, point):
        if point not in self.__dis:
            dis = 0
            self.__path = [point]
            while self.__path[-1] != self.__start:
                self.__path.append(self.__t[self.__path[-1]])
                dis = dis + \
                    self.__calculate_dis(self.__path[-1], self.__path[-2])
            self.__dis[point] = dis
        return self.__dis[point]

    def __parse_path(self):
        self.__path = [self.__end]
        while self.__path[-1] != self.__start:
            self.__path.append(self.__t[self.__path[-1]])

    def start_planing(self):
        """开始规划"""
        self.__dis = dict()
        self.__t = dict()
        self.__ed = set()
        self.__ed.add(self.__start)
        self.__x_current = self.__start

        for i in range(3000):
            self.__x_rand = self.__get_random_point()
            self.__x_near = self.__find_near_point_from_tree_for_star()
            # self.__x_near = self.__find_near_point_from_tree()
            self.__x_new = self.__calculate_new_node()
            self.__add_new_branch()
            self.__change_parent()
            if self.__is_arrived():
                self.__t[self.__end] = self.__x_new
                self.__parse_path()
                print(i)
                break
        else:
            print('未找到')
            return None
        return self.__path


# my_map = Map('./map/map_data_4.json', 'json')
# plan_rrt = PathPlanningRRTStar(my_map)
# print('========================================')
# points_rrt = plan_rrt.start_planing()
# path_a = my_map.calculate_path_distance(points_rrt)
# print('路径总长度', path_a)
# print('========================================')
# my_map.show_map('RRT', points=points_rrt)
