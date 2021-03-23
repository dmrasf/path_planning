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
        self.__radius = 2
        self.__size = my_map.get_size_grid()
        self.__rate = 0.5
        self.__branch_len = 2

    def __get_random_point(self):
        while True:
            if np.random.rand() < self.__rate:
                x = random.randint(0, len(self.__map[:, 0])-1)
                y = random.randint(0, len(self.__map[0, :])-1)
            else:
                x = self.__end[0]
                y = self.__start[1]
            if (x, y) not in self.__ed:
                break
        return (x, y)

    def __calculate_dis(self, p1, p2):
        p1 = self.__my_map.grid_to_real(p1)
        p2 = self.__my_map.grid_to_real(p2)
        return pow(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2), 0.5)

    def __find_near_point_from_tree_for_star(self):
        distance = float('inf')
        distance2 = float('inf')
        near = ()
        near2 = ()
        for node in self.__ed:
            tmp = self.__calculate_dis(node, self.__x_rand)
            if tmp > self.__radius:
                if tmp < distance2:
                    distance2 = tmp
                    near2 = node
                continue
            tmp = tmp + self.__calculate_path_dis(node)
            if tmp < distance:
                distance = tmp
                near = node
        if len(near) == 0:
            near = near2
        return near

    def __calculate_new_node(self):
        dis = self.__calculate_dis(self.__x_rand, self.__x_near)
        x_new = (self.__x_rand[0], self.__x_rand[1])
        x = self.__x_near[0] + (self.__x_rand[0] -
                                self.__x_near[0])*self.__branch_len/dis
        y = self.__x_near[1] + (self.__x_rand[1] -
                                self.__x_near[1])*self.__branch_len/dis
        x_new = (math.floor(x), math.floor(y))
        if x_new in self.__ed or x_new[0] < 0 or x_new[0] > self.__size[0]-1 or \
                x_new[1] < 0 or x_new[1] > self.__size[1] - 1 or \
                not self.__my_map.is_visible(self.__x_near, x_new):
            x_new = ()
        return x_new

    def __add_new_branch(self):
        self.__t[self.__x_new] = self.__x_near
        self.__ed.add(self.__x_new)

    def __change_parent(self):
        dis = self.__calculate_path_dis(self.__x_new)
        for node in self.__ed:
            if node == self.__x_new or node == self.__t[self.__x_new]:
                continue
            if self.__calculate_dis(node, self.__x_new) > self.__radius:
                continue
            if not self.__my_map.is_visible(self.__x_new, node):
                continue
            tmp = self.__calculate_dis(node, self.__x_new)
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
            path = [point]
            while path[-1] != self.__start:
                path.append(self.__t[path[-1]])
                dis = dis + \
                    self.__calculate_dis(path[-1], path[-2])
            self.__dis[point] = dis
        return self.__dis[point]

    def __parse_path(self):
        self.__path = [self.__end]
        while self.__path[-1] != self.__start:
            self.__path.append(self.__t[self.__path[-1]])

    def start_planing(self):
        """开始规划"""
        self.__t = dict()
        self.__ed = set()
        self.__ed.add(self.__start)
        self.__dis = dict()

        for i in range(100000):
            self.__x_rand = self.__get_random_point()
            self.__x_near = self.__find_near_point_from_tree_for_star()
            self.__x_new = self.__calculate_new_node()
            if len(self.__x_new) == 0:
                continue
            self.__add_new_branch()
            self.__change_parent()
            if self.__is_arrived():
                print(i)
                self.__t[self.__end] = self.__x_new
                self.__parse_path()
                break
        else:
            print('未找到')
            return None
        return self.__path


my_map = Map('./map/map_data_3.json', 'json')
plan_rrt_star = PathPlanningRRTStar(my_map)
print('========================================')
points_rrt_star = plan_rrt_star.start_planing()
path_rrt_star = my_map.calculate_path_distance(points_rrt_star)
print('路径总长度', path_rrt_star)
print('========================================')
my_map.show_map('RRT*', points=points_rrt_star)
