# 返回地图 255: 可以通过  20,0: 障碍
import json
import numpy as np
import math
import utils
# import time
# start = time.time()
# end = time.time()
# print(end-start)


def _getMapData(path):
    contents = dict()
    try:
        with open(path, 'r') as j:
            contents = json.loads(j.read())
        j.close()
    except:
        contents = None
    return contents


def _realToGrid(point_1, point_2, grid, shape):
    point_1_x = int(point_1[0]/grid)
    point_1_y = int(point_1[1]/grid)
    point_2_x = int(point_2[0]/grid)
    point_2_y = int(point_2[1]/grid)
    if point_1_x >= shape[0]:
        point_1_x = point_1_x - 1
    if point_2_x >= shape[0]:
        point_2_x = point_2_x - 1
    if point_1_y >= shape[1]:
        point_1_y = point_1_y - 1
    if point_2_y >= shape[1]:
        point_2_y = point_2_y - 1
    return point_1_x, point_1_y, point_2_x, point_2_y


def _drawLine(point_1, point_2, myMap, grid):
    point_1_x, point_1_y, point_2_x, point_2_y = _realToGrid(
        [point_1['pointX'], point_1['pointY']], [point_2['pointX'], point_2['pointY']], grid, myMap.shape)
    if point_1['lineType'] == 'straight':
        if abs(point_1_x-point_2_x) > abs(point_1_y-point_2_y):
            if point_1_x > point_2_x:
                x = np.array(range(point_2_x, point_1_x+1))
            else:
                x = np.array(range(point_1_x, point_2_x+1))
            y = (x-point_1_x)/(point_2_x-point_1_x) * \
                (point_2_y-point_1_y)+point_1_y
            y = np.rint(y)
        else:
            if point_1_y > point_2_y:
                y = np.array(range(point_2_y, point_1_y+1))
            else:
                y = np.array(range(point_1_y, point_2_y+1))
            x = (y-point_1_y)/(point_2_y-point_1_y) * \
                (point_2_x-point_1_x)+point_1_x
            x = np.rint(x)
        for i in range(len(x)):
            myMap[int(x[i]), int(y[i])] = 1
    elif point_1['lineType'] == 'curve':
        s = 0.005
        pass


def _drawBarrier(myMap, barrier, grid):
    for i in range(len(barrier)):
        _drawLine(barrier[i], barrier[(i+1) % len(barrier)], myMap, grid)


def _fillHole(x, y, myMap, old, new):
    fillingQueue = [[x, y]]
    filledSet = set()
    while len(fillingQueue) > 0:
        [x, y] = fillingQueue.pop()
        filledSet.add((x, y))
        if (x >= 0 and x < myMap.shape[0] and y >= 0 and y < myMap.shape[1] and myMap[x, y] == old):
            myMap[x, y] = new
            if (x+1, y) not in filledSet:
                fillingQueue.insert(0, [x+1, y])
            if (x-1, y) not in filledSet:
                fillingQueue.insert(0, [x-1, y])
            if (x, y+1) not in filledSet:
                fillingQueue.insert(0, [x, y+1])
            if (x, y-1) not in filledSet:
                fillingQueue.insert(0, [x, y-1])
    myMap[myMap == 1] = 1
    myMap[myMap == 0] = 1
    myMap[myMap == 2] = 0


def _expansionMap(needExpansionGrid, myMap):
    mask = needExpansionGrid + 1
    tmpMap = np.ones((myMap.shape[0]+2, myMap.shape[1]+2))
    tmpMap[1:myMap.shape[0]+1, 1:myMap.shape[1]+1] = myMap
    tmpMap2 = tmpMap.copy()
    [x, y] = tmpMap2.shape
    for i in range(x):
        for j in range(y):
            if i + mask > x or j + mask > y:
                continue
            if np.sum(tmpMap2[i:i+mask, j:j+mask]) > 0:
                tmpMap[i:i+mask, j:j+mask] = 1
    tmpMap[tmpMap == 0] = 255
    tmpMap[tmpMap2 == 1] = 0
    tmpMap[tmpMap == 1] = 0
    return tmpMap[1:myMap.shape[0]+1, 1:myMap.shape[1]+1]


def _mapProcess(myMap, robotSize, grid):
    _fillHole(0, 0, myMap, 0, 2)
    needExpansionGrid = math.ceil((robotSize-grid)/2/grid)
    if needExpansionGrid < 0:
        needExpansionGrid = 0
    return _expansionMap(needExpansionGrid, myMap)


def _buildMap(mapData):
    width = mapData['width']
    heigth = mapData['heigth']
    grid = mapData['grid']
    robotSize = mapData['robotSize']
    barriers = mapData['barriers']
    w_grids = math.ceil(width/grid)
    h_grids = math.ceil(heigth/grid)
    myMap = np.zeros((h_grids, w_grids), dtype='uint8')
    for barrier in barriers:
        _drawBarrier(myMap, barrier, grid)
    return _mapProcess(myMap, robotSize, grid)


def getMapFromData(path):
    mapData = _getMapData(path)
    if mapData is None:
        print('地图数据错误')
        return None
    myMap = _buildMap(mapData)
    p = _realToGrid(mapData['start'], mapData['end'],
                    mapData['grid'], myMap.shape)
    return myMap, (p[0], p[1]), (p[2], p[3])


def getVPointFromMap(path):
    pass
