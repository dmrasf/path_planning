import json
import numpy as np
import math
import utils


def _getMapData(path):
    # 从json文件获取地图数据
    contents = dict()
    try:
        with open(path, 'r') as j:
            contents = json.loads(j.read())
        j.close()
    except:
        contents = None
    return contents


def _realToGrid(point_1, point_2, grid, shape):
    # 实际点变为栅格点
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


def getPointsFromPoint1ToPoint2Line(point_1, point_2):
    # 栅格坐标点
    point_1_x, point_1_y, point_2_x, point_2_y = point_1[0], point_1[1], point_2[0], point_2[1]
    if abs(point_1_x-point_2_x) > abs(point_1_y-point_2_y):
        if point_1_x > point_2_x:
            x = np.array(range(point_2_x, point_1_x+1))
        else:
            x = np.array(range(point_1_x, point_2_x+1))
        y = (x-point_1_x)/(point_2_x-point_1_x) * \
            (point_2_y-point_1_y)+point_1_y
    else:
        if point_1_y > point_2_y:
            y = np.array(range(point_2_y, point_1_y+1))
        else:
            y = np.array(range(point_1_y, point_2_y+1))
        x = (y-point_1_y)/(point_2_y-point_1_y) * \
            (point_2_x-point_1_x)+point_1_x
    x = x.astype(np.int)
    y = y.astype(np.int)
    return [x, y]


def _azimuthAngle(x1,  y1,  x2,  y2):
    angle = 0.0
    dx = x2 - x1
    dy = y2 - y1
    if x2 == x1:
        angle = math.pi / 2.0
        if y2 == y1:
            angle = 0.0
        elif y2 < y1:
            angle = 3.0 * math.pi / 2.0
    elif x2 > x1 and y2 > y1:
        angle = math.atan(dx / dy)
    elif x2 > x1 and y2 < y1:
        angle = math.pi / 2 + math.atan(-dy / dx)
    elif x2 < x1 and y2 < y1:
        angle = math.pi + math.atan(dx / dy)
    elif x2 < x1 and y2 > y1:
        angle = 3.0 * math.pi / 2.0 + math.atan(dy / -dx)
    return (angle * 180 / math.pi)


def _getPointsFromPoint1ToPoint2Curve(point_1, point_2, radius, grid, direction, curvature):
    # 真实坐标点 半径  栅格大小  方向  曲率大小
    x1 = point_1[0]
    y1 = point_1[1]
    x2 = point_2[0]
    y2 = point_2[1]
    m, n = (x1+x2)/2, (y1+y2)/2
    k = -1/(y1-y2)*(x1-x2)
    a = k*k+1
    b = 2*k*(-k*m+n-y1)-2*x1
    c = x1*x1+(-k*m+n-y1)*(-k*m+n-y1)-radius*radius
    judge = b*b-4*a*c
    if judge < 0:
        return None
    x_1 = (-b+math.sqrt(judge))/2/a
    y_1 = k*(x_1-m)+n
    x_2 = (-b-math.sqrt(judge))/2/a
    y_2 = k*(x_2-m)+n
    anglel = _azimuthAngle(x1, y1, x_1, y_1)
    angle2 = _azimuthAngle(x_1, y_1, x2, y2)
    if angle2 > anglel:
        if direction == 'Counterclockwise':
            o_x = x_1
            o_y = y_1
        else:
            o_x = x_2
            o_y = y_2
    else:
        if direction == 'Counterclockwise':
            o_x = x_2
            o_y = y_2
        else:
            o_x = x_1
            o_y = y_1
    if curvature == 'small':
        if o_x == x_1 and o_y == y_1:
            o_x = x_2
            o_y = y_2
        else:
            o_x = x_1
            o_y = y_1
    print(o_x, o_y)
    return []


_getPointsFromPoint1ToPoint2Curve(
    [1, 1], [4, 4], 3, 0.05, 'Counterclockwise', 'big')


def _drawLine(point_1_data, point_2_data, myMap, grid):
    # 给定两个实际坐标点在栅格地图上绘制线条，目前只能绘制直线
    point_1, point_2 = [point_1_data['pointX'], point_1_data['pointY']], [
        point_2_data['pointX'], point_2_data['pointY']]
    point_1_x, point_1_y, point_2_x, point_2_y = _realToGrid(
        point_1, point_2, grid, myMap.shape)
    if point_1_data['lineType'] == 'straight':
        points = getPointsFromPoint1ToPoint2Line(
            [point_1_x, point_1_y], [point_2_x, point_2_y])
        myMap[points[0], points[1]] = 0
    elif point_1_data['lineType'] == 'curve':
        radius = point_1_data['radius']
        direction = point_1_data['direction']
        curvature = point_1_data['curvature']
        points = _getPointsFromPoint1ToPoint2Curve(
            point_1, point_2, radius, grid, direction, curvature)
        myMap[points[0], points[1]] = 0


def _drawBarrier(myMap, barrier, grid):
    # 绘制障碍
    for i in range(len(barrier)):
        _drawLine(barrier[i], barrier[(i+1) % len(barrier)], myMap, grid)


def _fillHole(x, y, myMap):
    # 填充空洞，4个区域连通
    fillingQueue = [[x, y]]
    filledSet = set()
    while len(fillingQueue) > 0:
        [x, y] = fillingQueue.pop()
        filledSet.add((x, y))
        if (x >= 0 and x < myMap.shape[0] and y >= 0 and y < myMap.shape[1] and myMap[x, y] == 255):
            myMap[x, y] = 1
            if (x+1, y) not in filledSet:
                fillingQueue.insert(0, [x+1, y])
            if (x-1, y) not in filledSet:
                fillingQueue.insert(0, [x-1, y])
            if (x, y+1) not in filledSet:
                fillingQueue.insert(0, [x, y+1])
            if (x, y-1) not in filledSet:
                fillingQueue.insert(0, [x, y-1])
    myMap[myMap == 255] = 0
    myMap[myMap == 1] = 255


def _expansionMap(needExpansionGrid, myMap, e):
    # 障碍向外扩展任意层数，需要0-255二值图
    mask = needExpansionGrid + 1
    tmpMap = myMap.copy()
    [x, y] = tmpMap.shape
    tmpMap2 = myMap.copy()
    for i in range(x):
        for j in range(y):
            if i + mask > x or j + mask > y:
                continue
            if np.min(tmpMap[i:i+mask, j:j+mask]) == 0:
                myMap[i:i+mask, j:j+mask] = e
    myMap[tmpMap2 == 0] = 0
    return myMap


def _mapProcess(myMap, robotSize, grid, e):
    # 图像处理
    _fillHole(0, 0, myMap)
    needExpansionGrid = math.ceil((robotSize-grid)/2/grid)
    if needExpansionGrid < 0:
        needExpansionGrid = 0
    return _expansionMap(needExpansionGrid, myMap, e)


def _buildMap(mapData, e):
    width = mapData['width']
    heigth = mapData['heigth']
    grid = mapData['grid']
    robotSize = mapData['robotSize']
    barriers = mapData['barriers']
    w_grids = math.ceil(width/grid)
    h_grids = math.ceil(heigth/grid)
    myMap = np.zeros((h_grids, w_grids), dtype='uint8')
    myMap[myMap == 0] = 255
    for barrier in barriers:
        _drawBarrier(myMap, barrier, grid)
    return _mapProcess(myMap, robotSize, grid, e)


def getMapFromData(path):
    mapData = _getMapData(path)
    if mapData is None:
        print('地图数据错误')
        return None
    e = 150
    myMap = _buildMap(mapData, e)
    p = _realToGrid(mapData['start'], mapData['end'],
                    mapData['grid'], myMap.shape)
    return myMap, (p[0], p[1]), (p[2], p[3]), (0, e)


##########################################################################
##########################################################################
##########################################################################

def _getNextContorPoint(openSet, currentPoint):
    # 获取连接的任意一个点
    if (currentPoint[0]+1, currentPoint[1]) in openSet:
        return (currentPoint[0]+1, currentPoint[1])
    if (currentPoint[0], currentPoint[1]+1) in openSet:
        return (currentPoint[0], currentPoint[1]+1)
    if (currentPoint[0]-1, currentPoint[1]) in openSet:
        return (currentPoint[0]-1, currentPoint[1])
    if (currentPoint[0], currentPoint[1]-1) in openSet:
        return (currentPoint[0], currentPoint[1]-1)
    return None


def _getRightBottomContourNextPoint(openSet, shape, pairType):
    # 得到每个轮廓右下角点
    n = 0
    rightBottomPoint = ()
    currentPoint = ()
    for point in openSet:
        if point[0] * shape[0] + point[1] > n:
            n = point[0] * shape[0] + point[1]
            rightBottomPoint = point
    if pairType == 0:
        currentPoint = (rightBottomPoint[0]-1, rightBottomPoint[1])
    else:
        currentPoint = (rightBottomPoint[0], rightBottomPoint[1]-1)
    return rightBottomPoint, currentPoint


def _getContoursPoints(myMap, e, pairType=0):
    # 获取逆时针或顺时针排序的轮廓点，e为轮廓值
    pointsMap = myMap == e
    openSet = set()
    contours = []
    for i in range(pointsMap.shape[0]):
        for j in range(pointsMap.shape[1]):
            if pointsMap[i, j]:
                openSet.add((i, j))
    while True:
        rightBottomPoint, currentPoint = _getRightBottomContourNextPoint(
            openSet, myMap.shape, pairType)
        openSet.remove(rightBottomPoint)
        openSet.remove(currentPoint)
        contour = [rightBottomPoint]
        contour.append(currentPoint)
        while True:
            currentPoint = _getNextContorPoint(openSet, currentPoint)
            if currentPoint is None:
                contours.append(contour.copy())
                contour.clear()
                break
            contour.append(currentPoint)
            openSet.remove(currentPoint)
        if len(openSet) == 0:
            break
    return contours


def _getVPointFromContour(contour, myMap):
    v_points = []
    point_1 = 0
    for i in range(1, len(contour)+2):
        i = i % len(contour)
        points = getPointsFromPoint1ToPoint2Line(contour[point_1], contour[i])
        if np.min(myMap[points[0], points[1]]) == 0:
            v_points.append((i-1) % len(contour))
            point_1 = (i-1) % len(contour)
    points = []
    for v_point in v_points:
        points.append((contour[v_point][0], contour[v_point][1]))
    return points


def _gridToReal(point, grid):

    return (point[0]*grid+grid/2, point[1]*grid+grid/2)


def getVPointFromMap(path):
    # 得到可视点
    mapData = _getMapData(path)
    if mapData is None:
        print('地图数据错误')
        return None
    e = 120
    grid = mapData['grid']
    myMap = _buildMap(mapData, 0)
    newMap = myMap.copy()
    myMap = _expansionMap(1, myMap, e)
    p = _realToGrid(mapData['start'], mapData['end'],
                    mapData['grid'], myMap.shape)
    contours = _getContoursPoints(myMap, e, 0)
    v_points = [(p[0], p[1])]
    for contour in contours:
        v_points.extend(_getVPointFromContour(contour, myMap))
    v_points.append((p[2], p[3]))
    return newMap, v_points, grid

# getVPointFromMap('./map_data.json')
