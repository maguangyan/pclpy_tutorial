# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/7
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 06_min_cut_segmentation.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile('../../data/min_cut_segmentation_tutorial.pcd', cloud) == -1:
        print('Cloud reading failed.')
        exit(-1)

    # 滤波
    indices = pcl.vectors.Int()
    ps = pcl.filters.PassThrough.PointXYZ()
    ps.setInputCloud(cloud)
    ps.setFilterFieldName('z')
    ps.setFilterLimits(0.0, 1.0)
    ps.filter(indices)

    # 使用最小切割分割
    seg = pcl.segmentation.MinCutSegmentation.PointXYZ()
    seg.setInputCloud(cloud)
    seg.setIndices(indices)

    foreground_points = pcl.PointCloud.PointXYZ()
    point = pcl.point_types.PointXYZ()
    point.x = 68.97
    point.y = -18.55
    point.z = 0.57
    foreground_points.push_back(point)
    seg.setForegroundPoints(foreground_points)  # 对象中心点

    seg.setSigma(0.25)  # σ
    seg.setRadius(3.0433856)    # 平滑代价计算所需的对象半径
    seg.setNumberOfNeighbours(14)   # 构建图时要找到的邻居数
    seg.setSourceWeight(0.8)    # 设置前景惩罚

    clusters = pcl.vectors.PointIndices()
    seg.extract(clusters)

    print('Maximum flow is ', seg.getMaxFlow())

    # 可视化
    colored_cloud = seg.getColoredCloud()
    viewer = pcl.visualization.CloudViewer('Cluster viewer')
    viewer.showCloud(colored_cloud)
    while not viewer.wasStopped(10):
        pass