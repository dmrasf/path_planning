import cv2
import numpy as np


img = cv2.imread('./maps/ori.png')
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

ret, thresh = cv2.threshold(imgray, 127, 255, 0)
# cv2.CHAIN_APPROX_NONE，所有的边界点都会被存储
# cv2.CHAIN_APPROX_SIMPLE 只需要这条直线的两个端点
contours, hierarchy = cv2.findContours(
    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# img ： 原始图像  contours：轮廓  第几个  颜色  粗细
img = cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
