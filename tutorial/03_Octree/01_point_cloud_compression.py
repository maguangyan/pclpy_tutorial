# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_point_cloud_compression.py

# 点云压缩这一块儿，pclpy的支持不是很好，而且需要 openNI 设备，这一块儿先不做。

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny.pcd", cloud)

    out = pcl.PointCloud.PointXYZ()
    # pcl.io.lzfCompress(cloud, cloud.size(), out, cloud.size()*2)
    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0, 0, 0, v0)
    viewer.addPointCloud(cloud, "cloud", v0)  # 显示点云

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0, 0, 0, v0)
    viewer.addPointCloud(out, "out cloud", v0)  # 显示点云

    while not viewer.wasStopped():
        viewer.spinOnce(10)