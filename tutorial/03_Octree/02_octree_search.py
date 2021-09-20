# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 02_octree_search.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云数据
    cloud_size = 1000
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)

    resolution = 128.0

    octree = pcl.octree.OctreePointCloudSearch.PointXYZ(resolution)
    octree.setInputCloud(cloud)
    octree.addPointsFromInputCloud()

    searchPoint = pcl.point_types.PointXYZ()
    searchPoint.x = np.random.ranf(1) * 1024
    searchPoint.y = np.random.ranf(1) * 1024
    searchPoint.z = np.random.ranf(1) * 1024

    # 体素最近邻搜索
    pointIdxVec = pclpy.pcl.vectors.Int()

    if octree.voxelSearch(searchPoint, pointIdxVec) > 0:
        print('Neighbors within voxel search at (', searchPoint.x,
              '', searchPoint.y,
              '', searchPoint.z, ')\n')
        for i in range(len(pointIdxVec)):
            print("  ", cloud.x[pointIdxVec[i]],
                  " ", cloud.y[pointIdxVec[i]],
                  " ", cloud.z[pointIdxVec[i]], "\n")

    # k最近邻搜索
    k = 10
    pointIdxNKNSearch = pclpy.pcl.vectors.Int()
    pointNKNSquaredDistance = pclpy.pcl.vectors.Float()

    print('K nearest neighbor search at (', searchPoint.x,
          '', searchPoint.y,
          '', searchPoint.z,
          ') with k =', k, '\n')
    if octree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0:
        for i in range(len(pointIdxNKNSearch)):
            print("  ", cloud.x[pointIdxNKNSearch[i]],
                  " ", cloud.y[pointIdxNKNSearch[i]],
                  " ", cloud.z[pointIdxNKNSearch[i]],
                  " (squared distance: ", pointNKNSquaredDistance[i], ")", "\n")

    # 半径最近邻搜索
    pointIdxRadiusSearch = pclpy.pcl.vectors.Int()
    pointRadiusSquaredDistance = pclpy.pcl.vectors.Float()

    radius = np.random.ranf(1) * 256.0
    print("Neighbors within radius search at (", searchPoint.x,
          " ", searchPoint.y, " ", searchPoint.z, ") with radius=",
          radius, '\n')
    if octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0:
        for i in range(len(pointIdxRadiusSearch)):
            print("  ", cloud.x[pointIdxRadiusSearch[i]],
                  " ", cloud.y[pointIdxRadiusSearch[i]],
                  " ", cloud.z[pointIdxRadiusSearch[i]],
                  " (squared distance: ", pointRadiusSquaredDistance[i], ")", "\n")
