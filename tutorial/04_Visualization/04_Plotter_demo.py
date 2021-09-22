# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 04_Plotter.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

if __name__ == '__main__':
    # 定义一个plotter
    plotter = pcl.visualization.PCLPlotter()

    # 定义一个多项式函数，y=x^2
    # func1 = np.array([0, 1, 0])
    # plotter.addPlotData(func1, -10, 10, "y = x^2")
    data = pcl.vectors.Float([1, 2, 3, 3, 4, 5, 6, 6, 6])
    plotter.addHistogramData(data, 10, "Histogram")
    plotter.setBackgroundColor(1, 1, 1)
    # 显示结果
    plotter.plot()
