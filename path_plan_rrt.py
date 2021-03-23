import numpy as np
import matplotlib.pyplot as plt
from my_map import Map
import json
import random
import math


class PathPlanningRRT():
    """RRT算法"""

    def __init__(self, my_map: Map):
        self.__my_map = my_map
        self.__map = my_map.get_map()
        self.__start = tuple(my_map.real_to_grid(my_map.get_start_point()))
        self.__end = tuple(my_map.real_to_grid(my_map.get_end_point()))
        self.__branch_len = 100

    def __get_random_point(self):
        while True:
            x = random.randint(0, len(self.__map[:, 0])-1)
            y = random.randint(0, len(self.__map[0, :])-1)
            if (x, y) not in self.__ed and self.__map[x, y] == 255:
                if self.__my_map.is_visible(self.__x_current, [x, y]):
                    return (x, y)

    def __calculate_dis(self, p1, p2):
        return pow(pow(p1[0]-p2[0], 2) + pow(p1[1]-p2[1], 2), 0.5)

    def __find_near_point_from_tree(self):
        distance = 10000000000000
        for node in self.__tree:
            tmp = self.__calculate_dis(node[0], self.__x_rand)
            try:
                if tmp < distance and self.__my_map.is_visible(node[0], self.__x_rand):
                    distance = tmp
                    self.__x_near = node[0]
            except Exception as e:
                print(node[0], self.__x_rand)
                print(e)
                raise e

    def __calculate_new_node(self):
        dis = self.__calculate_dis(self.__x_rand, self.__x_near)
        self.__x_new = (self.__x_rand[0], self.__x_rand[1])
        if dis > self.__branch_len:
            x = self.__x_near[0] + (self.__x_rand[0] -
                                    self.__x_near[0])*self.__branch_len/dis
            y = self.__x_near[1] + (self.__x_rand[1] -
                                    self.__x_near[1])*self.__branch_len/dis
            self.__x_new = (math.ceil(x), math.ceil(y))

    def __add_new_branch(self):
        self.__x_current = [self.__x_new[0], self.__x_new[1]]
        self.__tree.append([self.__x_new, self.__x_near])
        self.__ed.add(self.__x_new)

    def __is_arrived(self):
        if self.__my_map.is_visible(self.__x_new, self.__end) and \
                self.__calculate_dis(self.__x_new, self.__end) <= self.__branch_len:
            return True
        return False

    def __parse_path(self):
        self.__path_route = [self.__end]
        while self.__path_route[-1] != self.__start:
            for node in self.__tree:
                if node[0] == self.__path_route[-1]:
                    self.__path_route.append(node[1])

    def start_planing(self):
        """开始规划"""
        self.__tree = [[self.__start, self.__start]]
        self.__ed = set(self.__start)
        self.__x_current = self.__start

        for _ in range(10000):
            self.__x_rand = self.__get_random_point()
            self.__find_near_point_from_tree()
            self.__calculate_new_node()
            self.__add_new_branch()
            if self.__is_arrived():
                self.__tree.append([self.__end, self.__x_new])
                self.__parse_path()
                break
        return self.__path_route
