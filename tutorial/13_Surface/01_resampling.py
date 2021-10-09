# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/8
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_resampling.py


import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云数据
    cloud = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile('../../data/bun0.pcd', cloud) < 0:
        print('Error loading model cloud.')
        exit(-1)

    # 创建一个 KD-Tree
    tree = pcl.search.KdTree.PointXYZ()

    # 输出具有PointNormal类型，以便存储ML计算的法线S
    mls_points = pcl.PointCloud.PointNormal()

    # 初始化对象(第二点类型是法线类型，即使未使用)
    mls = pcl.surface.MovingLeastSquares.PointXYZ_PointNormal()

    mls.setComputeNormals(True)

    # 设置参数
    mls.setInputCloud(cloud)
    mls.setPolynomialOrder(2)
    mls.setSearchMethod(tree)
    mls.setSearchRadius(0.03)

    # 重建
    mls.process(mls_points)

    # 保存结果
    pcl.io.savePCDFile('bun0_mls.pcd', mls_points)
