import random
from my_map import Map
import numpy as np
np.seterr(divide='ignore', invalid='ignore')


class PathPlanningAnt(object):
    """蚁群算法"""

    def __init__(self, my_map: Map):
        self.__map = my_map
        self.__ants_num = 10
        # 信息素权重
        self.__a = 0.5
        # 路径长度权重
        self.__b = 1
        # 信息素挥发速度
        self.__p = 1
        # 每只蚂蚁所含有的信息素
        self.__pheromone = 10
        # 路径信息素初始值
        self.__init_phermonone = 2
        # 迭代次数，蚂蚁代数
        self.__iteration_num = 10

    def __calculate_graph(self):
        """计算可视点间距离"""
        start_point = self.__map.real_to_grid(self.__map.get_start_point())
        end_point = self.__map.real_to_grid(self.__map.get_end_point())
        visual_points = self.__map.get_visual_points()
        visual_points.append(end_point)
        visual_points.insert(0, start_point)
        self.__visual_points = visual_points
        len_visual_points = len(visual_points)
        self.__start = 0
        self.__end = len_visual_points-1
        self.__visual_graph = np.zeros((len_visual_points, len_visual_points))
        for i in range(len_visual_points):
            for j in range(len_visual_points):
                if visual_points[i] == visual_points[j]:
                    self.__visual_graph[i, j] = 0
                elif self.__map.is_visible(visual_points[i], visual_points[j]):
                    self.__visual_graph[i, j] = pow(pow(visual_points[i][0]-visual_points[j][0], 2) +
                                                    pow(visual_points[i][1]-visual_points[j][1], 2), 0.5)
                else:
                    self.__visual_graph[i, j] = -1
                self.__visual_graph[j, i] = self.__visual_graph[i, j]

    def set_params(self, ants_num=None, a=None, b=None, p=None, phermomone=None, init_phermomone=None, iteration_num=None):
        """设置算法参数 默认参数"""
        if ants_num is not None:
            self.__ants_num = ants_num
        if a is not None:
            self.__a = a
        if b is not None:
            self.__b = b
        if p is not None:
            self.__p = p
        if phermomone is not None:
            self.__pheromone = phermomone
        if init_phermomone is not None:
            self.__init_pheromone = init_phermomone
        if iteration_num is not None:
            self.__iteration_num = iteration_num

    def __init_path_phermonone(self):
        """构建信息素图"""
        self.__path_phermonone = \
            np.zeros((len(self.__visual_points), (len(self.__visual_points))))
        self.__path_phermonone[self.__path_phermonone == 0] = \
            self.__init_phermonone
        # 路径不可达则信息素设为0
        self.__path_phermonone[self.__visual_graph == 0] = 0
        self.__path_phermonone[self.__visual_graph == -1] = 0

    def __set_ants_position(self):
        """设置每一代蚂蚁位置，除终点外"""
        self.__current_iteration_ants = []
        for _ in range(self.__ants_num):
            self.__current_iteration_ants.append(
                [random.randint(0, len(self.__visual_points)-2)])

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
                ant.append(-1)
        return is_all_done

    def start_planing(self):
        """开始规划"""
        self.__calculate_graph()
        self.__init_path_phermonone()
        for _ in range(self.__iteration_num):
            self.__set_ants_position()
            while True:
                is_all_arrive_end = self.__select_next_pos_for_ants()
                if is_all_arrive_end:
                    break
            print(self.__current_iteration_ants)
            break


p = PathPlanningAnt(Map('./map_data_2.json'))
p.start_planing()