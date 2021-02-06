from PIL import Image
import matplotlib.pyplot as plt
import numpy as np


def get_map(fileName):
    """获取地图

    Args:
        fileName: 文件名
    Returns:
        地图
    """
    img = Image.open(fileName)
    n = np.array(img)

    H = []
    W = []
    h, w = np.shape(n)
    f = 0
    for i in range(h):
        if np.size(np.where(n[i] == 0)) > 0 and f == 0:
            f = 1
            H.append(i)
        if np.size(np.where(n[i] == 0)) == 0 and f == 1:
            f = 2
            H.append(i)
    f = 0
    for i in range(w):
        if np.size(np.where(n[:, i] == 0)) > 0 and f == 0:
            f = 1
            W.append(i)
        if np.size(np.where(n[:, i] == 0)) == 0 and f == 1:
            f = 2
            W.append(i)

    return n[H[0]:H[1], W[0]:W[1]]


def show_map(n):
    """显示地图

    Args:
        n: 地图
    """
    plt.imshow(n)
    plt.xticks([])
    plt.yticks([])
    plt.set_cmap('gray')
    plt.show()
