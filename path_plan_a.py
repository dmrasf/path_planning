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
            if self.__visual_graph[current_point[0], i] == -1:
                continue
            for close_point in self.__close_point_set:
                if i == close_point[0]:
                    break
            else:
                for open_point in self.__open_point_set:
                    if i == open_point[0]:
                        break
                else:
                    self.__open_point_set.add((i, current_point[0]))

    def __get_visual_graph(self):
        """计算可视点间距离"""
        len_visual_points = len(self.__visual_points)
        self.__visual_graph = np.zeros(
            (len_visual_points, len_visual_points))
        for i in range(len_visual_points):
            for j in range(i, len_visual_points):
                if self.__visual_points[i] == self.__visual_points[j]:
                    self.__visual_graph[i, j] = 0
                elif self.__map.is_visible(self.__visual_points[i], self.__visual_points[j]):
                    self.__visual_graph[i, j] = pow(pow(self.__visual_points[i][0]-self.__visual_points[j][0], 2) +
                                                    pow(self.__visual_points[i][1]-self.__visual_points[j][1], 2), 0.5)
                else:
                    self.__visual_graph[i, j] = -1
                self.__visual_graph[j, i] = self.__visual_graph[i, j]

    def __find_next_point(self, current_point):
        """找到下一个移动坐标"""
        min_distance = float('inf')
        best_point = []
        for tmp_point in self.__open_point_set:
            # 启发函数 f = h + g
            h = pow(pow(self.__visual_points[-1][0]-self.__visual_points[tmp_point[0]][0], 2) +
                    pow(self.__visual_points[-1][1]-self.__visual_points[tmp_point[0]][1], 2), 0.5)
            g = pow(pow(self.__visual_points[current_point[0]][0]-self.__visual_points[tmp_point[0]][0], 2) +
                    pow(self.__visual_points[current_point[0]][1]-self.__visual_points[tmp_point[0]][1], 2), 0.5) * 1.5
            f = h + g
            if f < min_distance:
                min_distance = f
                best_point = tmp_point
        return best_point

    def __parse_path_from_close_set(self, end, is_optimising=False):
        """从访问过的点集中解析出一条路径"""
        path_route = [end]
        tmp_point = end
        while True:
            for points in self.__close_point_set:
                if points[0] == tmp_point:
                    path_route.append(points[1])
                    tmp_point = points[1]
            if tmp_point == 0:
                break
        path_route.reverse()
        self.__path_route = []
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
        self.__close_point_set = set()
        self.__open_point_set = set()
        start = 0
        end = len(self.__visual_points) - 1
        current_point = (start, start)
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
            if current_point[0] == end:
                break
        self.__close_point_set.remove((start, start))
        return self.__parse_path_from_close_set(end, is_optimising=is_optimising)
