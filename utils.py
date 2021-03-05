from PIL import Image
import matplotlib.pyplot as plt
import numpy as np


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
