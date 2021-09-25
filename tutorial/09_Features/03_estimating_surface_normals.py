# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/24
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 03_estimating_surface_normals.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

if __name__ == '__main__':
    # 生成点云数据
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny.pcd", cloud)
    print(cloud.size())
    # 构造法线估计类
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)

    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)

    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.03)
    # 计算法线
    ne.compute(cloud_normals)
    print(cloud_normals.size())
