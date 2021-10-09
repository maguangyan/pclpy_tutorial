# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/9
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 03_greedy_projection.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云数据
    cloud = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile('../../data/bun0.pcd', cloud) < 0:
        print('Error loading model cloud.')
        exit(-1)

    # 法线估计 normals不包含点，只包含法线+曲面曲率
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    normals = pcl.PointCloud.Normal()
    tree = pcl.search.KdTree.PointXYZ()
    tree.setInputCloud(cloud)
    ne.setInputCloud(cloud)
    ne.setSearchMethod(tree)
    ne.setKSearch(20)
    ne.compute(normals)

    # 连接XYZ 和 normal 字段
    cloud_with_normals = pcl.PointCloud.PointNormal(np.hstack((
        cloud.xyz, normals.normals, normals.curvature.reshape(-1, 1))))

    # 开始准备贪婪三角化算法
    tree2 = pcl.search.KdTree.PointNormal()
    tree2.setInputCloud(cloud_with_normals)

    gp3 = pcl.surface.GreedyProjectionTriangulation.PointNormal()
    triangles = pcl.PolygonMesh()

    # 设置连接点之间的最大距离(最大边长)
    gp3.setSearchRadius(0.025)
    # 设置参数
    gp3.setMu(2.5)
    gp3.setMaximumNearestNeighbors(100)
    gp3.setMaximumSurfaceAngle(np.pi / 4)  # 45 degrees
    gp3.setMinimumAngle(np.pi / 18)  # 10degrees
    gp3.setMaximumAngle(2 * np.pi / 3)  # 120 degrees
    gp3.setNormalConsistency(False)

    # 获取结果并保存
    gp3.setInputCloud(cloud_with_normals)
    gp3.setSearchMethod(tree2)
    gp3.reconstruct(triangles)

    save = pcl.io.saveVTKFile('bun0_gp3.vtk', triangles)

    # 顶点信息
    parts = pcl.vectors.Int()
    states = pcl.vectors.Int()
