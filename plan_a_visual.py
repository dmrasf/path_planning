# Purpose: 可视图测试A*算法
# Date: 2021-01-24
# Author: dmrasf
# Version: 0.2

import numpy as np
from utils import get_map, show_map


def get_v_points(start, end, n):
    # 这些可视点需要程序算出，这里先手写
    return [start, (0, 1), (2, 1), (5, 1), (7, 1), (0, 9), (7, 9), (0, 14), (2, 14), (5, 11), (7, 11), (0, 19), (7, 19), end]


def update_open_set(open_point_set, close_point_set, current_point, v_g):
    """更新待访问集合

    Args:
        open_point_set: 待访问点集合
        close_point_set: 已访问点集合
        current_point: 机器人当前所在位置
        v_g: 可视图
    """
    for i in range(len(v_g)):
        if v_g[current_point[0], i] == -1:
            continue
        for close_point in close_point_set:
            if i == close_point[0]:
                break
        else:
            for open_point in open_point_set:
                if i == open_point[0]:
                    break
            else:
                open_point_set.add((i, current_point[0]))


def update_close_set(close_point_set, current_point):
    """更新已访问集合

    Args:
        close_point_set: 已访问点集合
        current_point: 机器人当前所在位置
    """
    close_point_set.add(current_point)


def find_next_point(current_point, open_point_set, v_g,  v_points):
    """找到下一个移动坐标

    Args:
        current_point: 机器人当前所在位置
        open_point_set: 待访问点集合
        v_g: 可视图
        v_points: 可视点
    Returns:
        下一个点坐标
    """
    min_point = ()
    min_distance = float('inf')
    for tmp_point in open_point_set:
        # 启发函数
        tmp_distance = v_g[current_point[0]][tmp_point[0]] + \
            pow(pow(v_points[current_point[0]][0]-v_points[tmp_point[0]][0], 2) +
                pow(v_points[current_point[0]][1]-v_points[tmp_point[0]][1], 2), 0.5)
        if tmp_distance < min_distance:
            min_distance = tmp_distance
            min_point = tmp_point
    return min_point


def parse_points(close_point_set, start, end, v_points):
    """根据访问过的集合计算路径

    Args:
        close_point_set: 已访问点集合
        start: 起点
        end: 终点
        v_points: 可视点
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
    route_points = []
    for i in route:
        route_points.append(v_points[i])
    return route_points


def route_plan_a(v_g, v_points):
    """路径规划A*算法

    Args:
        v_g: 可视图
        v_points: 可视点
    Returns:
        可视点构成的点的列表
    """
    close_point_set = set()
    open_point_set = set()
    start = 0
    end = len(v_g) - 1
    current_point = (start, start)
    update_close_set(close_point_set, current_point)
    while True:
        update_open_set(open_point_set, close_point_set, current_point, v_g)
        current_point = find_next_point(
            current_point, open_point_set, v_g, v_points)
        try:
            open_point_set.remove(current_point)
        except Exception as _:
            print("没有可行路径")
            exit()
        update_close_set(close_point_set, current_point)
        if current_point[0] == end:
            break
    close_point_set.remove((start, start))
    return parse_points(close_point_set, start, end, v_points)


# 待完成
def optimising_path(points, v_g, v_points):
    return points


# 有点问题，需要栅格大小和机器人底座大小数据
def check_is_visible(p1, p2, n, barrier):
    """检查两个点是否连通

    Args:
        p1: 第一个点
        p2: 第二个点
        n: 地图
        barrier: 地图上障碍物值
    Returns:
        是否可视
    """
    if abs(p1[0]-p2[0]) > abs(p1[1]-p2[1]):
        if p1[0] > p2[0]:
            x = np.array(range(p2[0], p1[0]+1))
        else:
            x = np.array(range(p1[0], p2[0]+1))
        y = (x-p1[0])/(p2[0]-p1[0])*(p2[1]-p1[1])+p1[1]
        y = np.rint(y)
    else:
        if p1[1] > p2[1]:
            y = np.array(range(p2[1], p1[1]+1))
        else:
            y = np.array(range(p1[1], p2[1]+1))
        x = (y-p1[1])/(p2[1]-p1[1])*(p2[0]-p1[0])+p1[0]
        x = np.rint(x)

    for i in range(len(x)):
        if n[int(x[i]), int(y[i])] in barrier:
            return False
    return True


def solve_graph(v_points, n, barrier):
    """根据可视点求解可视图，点与点之间是否可视，-1为不可视，其他值为距离

    Args:
        v_points: 可视点
        n: 地图
        barrier: 地图上障碍物值
    Returns:
        可视图
    """
    len_point = len(v_points)
    v_g = np.zeros((len_point, len_point))
    for i in range(len_point):
        for j in range(i, len_point):
            if v_points[i] == v_points[j]:
                v_g[i, j] = 0
            elif check_is_visible(v_points[i], v_points[j], n, barrier):
                v_g[i, j] = pow(pow(v_points[i][0]-v_points[j][0], 2) +
                                pow(v_points[i][1]-v_points[j][1], 2), 0.5)
            else:
                v_g[i, j] = -1
            v_g[j, i] = v_g[i, j]
    return v_g


if __name__ == "__main__":
    # 从pgm文件得出
    n = get_map('./maps/map.pgm')
    n = np.array([[1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, ],
                  [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, ],
                  [2, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [3, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [4, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [5, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, ],
                  [6, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, ],
                  [7, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, ]])
    n[n != 0] = 255
    start = (4, 0)
    end = (4, 16)
    barrier = [0]
    v_points = get_v_points(start, end, n)
    v_g = solve_graph(v_points, n, barrier)
    points = route_plan_a(v_g, v_points)
    print(points)
    for p in points:
        n[p[0], p[1]] = 100
    show_map(n)
