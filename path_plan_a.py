import numpy as np
from my_map import Map
import json


class PathPlanningA():
    """A*算法"""

    def __init__(self, my_map: Map):
        self.__map = my_map
        self.__visual_graph = self.__map.get_visual_graph()
        self.__visual_points = self.__map.get_visual_points()

    def __update_close_set(self, current_point):
        """更新已访问集合"""
        self.__close_point_set.add(current_point)

    def __update_open_set(self, current_point):
        """更新待访问集合"""
        for i in range(len(self.__visual_points)):
            if self.__visual_graph[current_point, i] > 0 and i not in self.__open_point_set and i not in self.__close_point_set:
                self.__open_point_set.add(i)
                self.__tree[i] = current_point

    def __find_next_point(self, current_point):
        """找到下一个移动坐标"""
        min_distance = float('inf')
        best_point = []
        for tmp_point in self.__open_point_set:
            h = pow(pow(self.__visual_points[-1][0]-self.__visual_points[tmp_point][0], 2)*self.__map.get_grid() +
                    pow(self.__visual_points[-1][1]-self.__visual_points[tmp_point][1], 2)*self.__map.get_grid(), 0.5)
            g = self.__map.get_visual_graph()[current_point][tmp_point]
            f = h + g
            if f < min_distance:
                min_distance = f
                best_point = tmp_point
        return best_point

    def __parse_path_from_close_set(self, end, is_optimising=False):
        """从访问过的点集中解析出一条路径"""
        path_route = [end]
        while path_route[-1] != 0:
            path_route.append(self.__tree[path_route[-1]])
        for point in path_route:
            self.__path_route.append(self.__visual_points[point])
        if is_optimising:
            self.__path_route = self.__map.optimising_path(self.__path_route)
        return self.__path_route

    def save_route_path(self, save_path='point_a.json'):
        self.__map.save_route_path(save_path, self.__path_route)
        print('路径保存成功')

    def start_planing(self, is_optimising=False):
        """开始规划"""
        self.__tree = dict()
        self.__close_point_set = set()
        self.__open_point_set = set()
        self.__path_route = []
        start = 0
        end = len(self.__visual_points) - 1
        current_point = start
        self.__update_close_set(current_point)
        while True:
            self.__update_open_set(current_point)
            current_point = self.__find_next_point(current_point)
            try:
                self.__open_point_set.remove(current_point)
            except:
                print("没有可行路径")
                return None
            self.__update_close_set(current_point)
            if current_point == end:
                break
        return self.__parse_path_from_close_set(end, is_optimising=is_optimising)
