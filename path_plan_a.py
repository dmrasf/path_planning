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
        new_open_points = []
        for i in range(len(self.__visual_points)):
            if self.__visual_graph[current_point, i] > 0 \
                    and i not in self.__open_point_set \
                    and i not in self.__close_point_set:
                self.__open_point_set.add(i)
                self.__tree[i] = current_point
                new_open_points.append(i)
        return new_open_points

    def __find_next_point(self, current_point):
        """找到下一个移动坐标"""
        min_distance = float('inf')
        best_point = []
        for tmp_point in self.__open_point_set:
            h = pow(pow(self.__visual_points[-1][0]-self.__visual_points[tmp_point][0], 2)*self.__map.get_grid() +
                    pow(self.__visual_points[-1][1]-self.__visual_points[tmp_point][1], 2)*self.__map.get_grid(), 0.5)
            g = self.__visual_graph[current_point][tmp_point]
            f = h + g
            if f < min_distance:
                min_distance = f
                best_point = tmp_point
        return best_point

    def __parse_path_from_close_set(self, end):
        """从访问过的点集中解析出一条路径"""
        path_route = [end]
        while path_route[-1] != 0:
            path_route.append(self.__tree[path_route[-1]])
        for point in path_route:
            self.__path_route.append(self.__visual_points[point])
        return self.__path_route

    def save_route_path(self, save_path='point_a.json'):
        self.__map.save_route_path(save_path, self.__path_route)
        print('路径保存成功')

    def __calculate_path_distance(self, point, is_cal):
        if point not in self.__point_to_start_dis.keys() or is_cal:
            dis = 0
            path = [point]
            while path[-1] != 0:
                path.append(self.__tree[path[-1]])
                dis = dis + self.__visual_graph[path[-1], path[-2]]
            self.__point_to_start_dis[point] = dis
        return self.__point_to_start_dis[point]

    def __change_parent(self, new_open_points):
        """优化函数"""
        old_parent = set()
        for p in new_open_points:
            old_parent.add(self.__tree[p])
        tmp_parent = set.union(self.__close_point_set, self.__open_point_set)
        while True:
            for p in new_open_points:
                for parent in tmp_parent:
                    tmp_distance = self.__visual_graph[parent, p]
                    if tmp_distance <= 0:
                        continue
                    parent_distance = 0
                    if parent in new_open_points:
                        parent_distance = self.__calculate_path_distance(
                            parent, True)
                    else:
                        parent_distance = self.__calculate_path_distance(
                            parent, False)
                    if tmp_distance + parent_distance < self.__calculate_path_distance(p, False):
                        self.__tree[p] = parent
            new_parent = set()
            for p in new_open_points:
                new_parent.add(self.__tree[p])
            if len(set.difference(new_parent, old_parent)) == 0:
                break
            old_parent = new_parent.copy()

    def start_planing(self, is_optimising=False):
        """开始规划"""
        self.__tree = dict()
        self.__close_point_set = set()
        self.__open_point_set = set()
        self.__point_to_start_dis = dict()
        self.__path_route = []
        start = 0
        end = len(self.__visual_points) - 1
        current_point = start
        self.__update_close_set(current_point)
        while True:
            # {0, 64, 70, 13, 14, 15, 16, 17, 18, 52, 53, 56, 57, 58}
            new_open_points = self.__update_open_set(current_point)
            if is_optimising:
                self.__change_parent(new_open_points)
            current_point = self.__find_next_point(current_point)
            try:
                self.__open_point_set.remove(current_point)
            except:
                print("没有可行路径")
                return None
            self.__update_close_set(current_point)
            if is_optimising:
                if end in self.__tree.keys():
                    break
            else:
                if end in self.__close_point_set:
                    break
        return self.__parse_path_from_close_set(end)
