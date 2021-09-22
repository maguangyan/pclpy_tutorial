# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_cloud_viewer.py

import pclpy
from pclpy import pcl


if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny.pcd", cloud)

    # 可视化
    viewer = pcl.visualization.CloudViewer("viewer")
    viewer.showCloud(cloud, "sample cloud")  # 显示点云
    while not viewer.wasStopped(10):
        pass
