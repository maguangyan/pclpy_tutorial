# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/19
# @Author : yan
# @Email : 1792659158@qq.com
# @File : savePCD.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云
    a = np.random.ranf(300).reshape(-1, 3)
    cloud = pcl.PointCloud.PointXYZ.from_array(a)

    pcl.io.savePCDFileASCII("./save_pcd.pcd", cloud)
    print(cloud.size())

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    viewer.addPointCloud(cloud, "sample cloud")  # 显示点云
    while not viewer.wasStopped():
        viewer.spinOnce(10)