import random
from my_map import Map
import json
import numpy as np
np.seterr(divide='ignore', invalid='ignore')


class PathPlanningAnt(object):
    """蚁群算法"""

    def __init__(self, my_map: Map):
        self.__map = my_map
        self.__ants_num = 10
        # 信息素权重
        self.__a = 1
        # 路径长度权重
        self.__b = 0.5
        # 信息素挥发速度
        self.__p = 0.4
        # 每只蚂蚁所含有的信息素
        self.__ant_pheromone = 100
        # 路径信息素初始值
        self.__init_paht_phermonone_value = 2
        # 迭代次数，蚂蚁代数
        self.__iteration_num = 10
        self.__visual_graph = self.__map.get_visual_graph()
        self.__visual_points = self.__map.get_visual_points()
        self.__end = len(self.__visual_points)-1

    def set_params(self, ants_num=None, a=None, b=None, p=None, ant_phermomone=None, init_path_phermomone_value=None, iteration_num=None):
        """设置算法参数 默认参数"""
        if ants_num is not None:
            self.__ants_num = ants_num
        if a is not None:
            self.__a = a
        if b is not None:
            self.__b = b
        if p is not None:
            self.__p = p
        if ant_phermomone is not None:
            self.__ant_pheromone = ant_phermomone
        if init_path_phermomone_value is not None:
            self.__init_path_pheromone_value = init_path_phermomone_value
        if iteration_num is not None:
            self.__iteration_num = iteration_num

    def __init_path_phermonone(self):
        """构建信息素图"""
        self.__path_phermonone = np.zeros((len(self.__visual_points),
                                           (len(self.__visual_points))))
        self.__path_phermonone[self.__path_phermonone == 0] = \
            self.__init_paht_phermonone_value
        # 路径不可达则信息素设为0
        self.__path_phermonone[self.__visual_graph == 0] = 0
        self.__path_phermonone[self.__visual_graph == -1] = 0

    def __set_ants_position(self):
        """设置每一代蚂蚁位置，除终点外"""
        self.__current_iteration_ants = []
        for _ in range(self.__ants_num):
            self.__current_iteration_ants.append([0])

    def __update_path_phermonone(self):
        """更新路径信息素"""
        self.__path_phermonone = self.__path_phermonone*(1-self.__p)
        for ant in self.__current_iteration_ants:
            if ant[-1] != -1:
                path = 0
                for i in range(len(ant)-1):
                    path = path + self.__visual_graph[ant[i], ant[i+1]]
                delta_p = self.__ant_pheromone/path
                for i in range(len(ant)-1):
                    self.__path_phermonone[ant[i], ant[i+1]] = \
                        self.__path_phermonone[ant[i], ant[i+1]] + delta_p
                    self.__path_phermonone[ant[i+1], ant[i]] = \
                        self.__path_phermonone[ant[i], ant[i+1]]

    def __calculate_probability(self, point_1, point_2):
        """计算可行坐标的信息素与路径长度值"""
        if self.__visual_graph[point_1, point_2] == 0 or self.__visual_graph[point_1, point_2] == -1:
            return 0
        return pow(self.__path_phermonone[point_1, point_2], self.__a) * pow(1/self.__visual_graph[point_1, point_2], self.__b)

    def __select_next_pos_for_ants(self):
        """选择蚂蚁下一个坐标"""
        is_all_done = True
        for ant in self.__current_iteration_ants:
            if ant[-1] == self.__end or ant[-1] == -1:
                continue
            is_all_done = False
            point_to_selected = list(range(self.__end+1))
            for p_ed in ant:
                point_to_selected.remove(p_ed)
            probabilities = np.array(point_to_selected, dtype=float)
            for i in range(len(probabilities)):
                probabilities[i] = \
                    self.__calculate_probability(ant[-1], point_to_selected[i])
            probabilities = probabilities/np.sum(probabilities)
            try:
                ant.append(np.random.choice(
                    point_to_selected, 1, p=probabilities)[0])
            except:
                # 遇到死路设为-1，不会增加信息素
                ant.append(-1)
        return is_all_done

    def __get_final_path(self, is_optimising=False):
        """根据信息素多少求得路径"""
        path_route = [0]
        while True:
            tmp = []
            for i in range(len(self.__visual_points)):
                tmp.append(self.__calculate_probability(path_route[-1], i))
            sort_point = np.argsort(np.array(tmp))
            for point in sort_point[::-1]:
                if point not in path_route:
                    path_route.append(point)
                    break
            else:
                print('没有路径')
                return None
            if path_route[-1] == self.__end:
                break
        self.__path_route = []
        for point in path_route:
            self.__path_route.append(self.__visual_points[point])
        if is_optimising:
            self.__path_route = self.__map.optimising_path(self.__path_route)
        return self.__path_route

    def save_route_path(self, save_path='points_ant.json'):
        self.__map.save_route_path(save_path, self.__path_route)
        print('路径保存成功')

    def start_planing(self, is_optimising=False):
        """开始规划"""
        self.__init_path_phermonone()
        for _ in range(self.__iteration_num):
            self.__set_ants_position()
            while True:
                is_all_arrive_end = self.__select_next_pos_for_ants()
                if is_all_arrive_end:
                    break
            self.__update_path_phermonone()
        return self.__get_final_path(is_optimising=is_optimising)
