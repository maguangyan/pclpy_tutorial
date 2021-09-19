# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/19
# @Author : yan
# @Email : 1792659158@qq.com
# @File : readPCD.py

import pclpy
from pclpy import pcl

if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny.pcd", cloud)

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    viewer.addPointCloud(cloud, "sample cloud")  # 显示点云
    while not viewer.wasStopped():
        viewer.spinOnce(10)