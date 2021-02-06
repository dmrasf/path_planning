# Purpose: 栅格小地图测试A*算法
# Date: 2021-01-24
# Author: dmrasf
# Version: 0.2

import numpy as np
from utils import show_map, get_map


def cal_distance(n, end):
    """计算每个点到目的地距离

    Args:
        n: 地图 array
        end: 终点，元组类型
    Returns:
        与n大小相同的矩阵
    """
    n_distance = np.zeros(n.shape)
    for i in range(n_distance.shape[0]):
        for j in range(n_distance.shape[1]):
            n_distance[i][j] = pow(pow(end[0]-i, 2)+pow(end[1]-j, 2), 0.5)
    return n_distance


def update_open_set(open_point_set, close_point_set, current_point, n, barrier):
    """更新待访问集合

    Args:
        open_point_set: 待访问点集合
        close_point_set: 已访问点集合
        current_point: 机器人当前所在位置
        n: 地图
        barrier: 地图上障碍物值
    """
    w, h = n.shape
    for i in range(current_point[0][0]-1, current_point[0][0]+2):
        for j in range(current_point[0][1]-1, current_point[0][1]+2):
            if i < 0 or i >= w or j < 0 or j >= h:
                continue
            if i == current_point[0][0] and j == current_point[0][1]:
                continue
            if n[i][j] in barrier:
                continue
            for tmp_point in close_point_set:
                if (i, j) == tmp_point[0]:
                    break
            else:
                closed = ((i, j), (current_point[0][0], current_point[0][1]))
                open_point_set.add(closed)


def update_close_set(close_point_set, current_point):
    """更新已访问集合

    Args:
        close_point_set: 已访问点集合
        current_point: 机器人当前所在位置
    """
    close_point_set.add(current_point)


def find_next_point(current_point, open_point_set, n_distance):
    """找到下一个移动坐标

    Args:
        current_point: 机器人当前所在位置
        open_point_set: 待访问点集合
        n_distance: 点与点之间距离
    Returns:
        下一个点坐标
    """
    min_point = ()
    min_distance = float('inf')
    for tmp_point in open_point_set:
        # 启发函数
        tmp_distance = abs(tmp_point[0][0]-current_point[0][0]) + \
            abs(tmp_point[0][1]-current_point[0][1]) + \
            n_distance[tmp_point[0][0]][tmp_point[0][1]]
        if tmp_distance < min_distance:
            min_distance = tmp_distance
            min_point = tmp_point
    return min_point


def parse_points(close_point_set, start, end):
    """根据访问过的集合计算路径

    Args:
        close_point_set: 已访问点集合
        start: 起点
        end: 终点
    """
    route = [end]
    tmp = end
    while True:
        for point in close_point_set:
            if point[0] == tmp:
                route.append(point[1])
                tmp = point[1]
        if tmp == start:
            break
    return route


def route_plan_a(start, end, n, barrier):
    """路径规划A*算法

    Args:
        start: 起点
        end: 终点
        n: 地图
        barrier: 地图上障碍物值
    Returns:
        地图上从终点到起点的路径构成的点的列表
    """
    if start == end:
        return [start]
    # 访问过的坐标 ((子坐标), (父坐标))
    close_point_set = set()
    # 未访问过的坐标
    open_point_set = set()
    n_distance = cal_distance(n, end)

    # 当前坐标
    current_point = ((start[0], start[1]), (start[0], start[1]))
    # 添加访问过和待访问坐标
    update_close_set(close_point_set, current_point)
    while True:
        update_open_set(open_point_set, close_point_set,
                        current_point, n, barrier)
        # 从待访问集合中挑选出最合适下一个坐标，同时从集合中去除
        current_point = find_next_point(
            current_point, open_point_set, n_distance)
        try:
            open_point_set.remove(current_point)
        except Exception as _:
            print("没有可行路径")
            exit()
        update_close_set(close_point_set, current_point)
        if current_point[0][0] == end[0] and current_point[0][1] == end[1]:
            break
    close_point_set.remove((start, start))
    return parse_points(close_point_set, start, end)


def optimising_path(points, start, n, barrier):
    """优化路径

    Args:
        points: 有路径规划算法计算出的路径点
        start: 起点
        n: 地图
        barrier: 地图上障碍物值
    Returns:
        地图上从终点到起点的路径构成的点集
    """
    min_len = len(points)
    i = 0
    while i < len(points):
        tmp_points = route_plan_a(
            start, (points[i][0], points[i][1]), n, barrier)
        tmp_len = len(tmp_points) + i
        if min_len > tmp_len:
            min_len = tmp_len
            points[i: -1] = tmp_points
        i = i + 1
    return points


if __name__ == "__main__":
    n = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, ],
                  [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, ],
                  [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, ],
                  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, ]])
    n[n == 1] = 255
    start = (3, 3)
    end = (4, 16)
    barrier = [0]

    # n = get_map('./maps/map.pgm')
    # start = (80, 10)
    # end = (80, 200)
    # barrier = [0, 205]

    points = route_plan_a(start, end, n, barrier)
    points = optimising_path(points, start, n, barrier)
    for i in points:
        n[i[0]][i[1]] = 100

    show_map(n)
