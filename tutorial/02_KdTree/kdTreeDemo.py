# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : kdTreeDemo.py


import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云数据
    cloud_size = 100
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)

    kdtree = pcl.kdtree.KdTreeFLANN.PointXYZ()
    kdtree.setInputCloud(cloud)
    searchPoint = pcl.point_types.PointXYZ()
    searchPoint.x = np.random.ranf(1) * 1024
    searchPoint.y = np.random.ranf(1) * 1024
    searchPoint.z = np.random.ranf(1) * 1024
    # k最近邻搜索
    k = 8
    pointIdxNKNSearch = pclpy.pcl.vectors.Int([0] * k)
    pointNKNSquaredDistance = pclpy.pcl.vectors.Float([0] * k)
    print('K nearest neighbor search at (', searchPoint.x,
          '', searchPoint.y,
          '', searchPoint.z,
          ') with k =', k, '\n')
    if kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0:
        for i in range(len(pointIdxNKNSearch)):
            print("  ", cloud.x[pointIdxNKNSearch[i]],
                  " ", cloud.y[pointIdxNKNSearch[i]],
                  " ", cloud.z[pointIdxNKNSearch[i]],
                  " (squared distance: ", pointNKNSquaredDistance[i], ")", "\n")

    # 使用半径最近邻搜索
    pointIdxNKNSearch = pclpy.pcl.vectors.Int()
    pointNKNSquaredDistance = pclpy.pcl.vectors.Float()

    radius = np.random.ranf(1) * 256.0
    print("Neighbors within radius search at (", searchPoint.x,
          " ", searchPoint.y, " ", searchPoint.z, ") with radius=",
          radius, '\n')
    if kdtree.radiusSearch(searchPoint, radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0:
        for i in range(len(pointIdxNKNSearch)):
            print("  ", cloud.x[pointIdxNKNSearch[i]],
                  " ", cloud.y[pointIdxNKNSearch[i]],
                  " ", cloud.z[pointIdxNKNSearch[i]],
                  " (squared distance: ", pointNKNSquaredDistance[i], ")", "\n")
