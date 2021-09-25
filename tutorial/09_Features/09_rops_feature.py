# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 09_rops_feature.py


import pclpy
from pclpy import pcl
import numpy as np
import sys
import matplotlib.pyplot as plt
import pyvista as pv


def show_cloud(cloud):
    # 使用pyvista进行三角化
    cloud = pv.PolyData(cloud.xyz)
    mesh = cloud.delaunay_2d(tol=1e-05, alpha=0, offset=1.0, bound=False,
                             inplace=False, edge_source=None, progress_bar=False)
    # Establish geometry within a pv.Plotter()
    p = pv.Plotter()
    # p.add_mesh(mesh.smooth(n_iter=100), color=True)
    p.add_mesh(cloud, color=True, point_size=1, scalars=mesh.points[:, 2], cmap='viridis')
    p.camera_position = 'xy'
    p.show_bounds()
    p.show()


if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/min_cut_segmentation_tutorial.pcd", cloud)
    # show_cloud(cloud)

    indices = pcl.PointIndices()
    indices.indices = pcl.vectors.Int(np.arange(0, cloud.size()))

    # 构造法线估计类
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)

    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)

    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.3)

    # 计算法线
    ne.compute(cloud_normals)

    # 连接XYZ和normal字段
    cloud_with_normals = pcl.PointCloud.PointNormal().from_array(np.hstack((
        cloud.xyz, cloud_normals.normals, cloud_normals.curvature.reshape(-1, 1))))

    # 构造查询树
    tree = pcl.search.KdTree.PointNormal()
    tree.setInputCloud(cloud_with_normals)

    # 贪婪三角化
    gp3 = pcl.surface.GreedyProjectionTriangulation.PointNormal()
    triangles = pcl.PolygonMesh()

    # 设置参数
    gp3.setSearchRadius(0.1)  # 设置连接点之间的最大距离，（即是三角形最大边长）
    gp3.setMu(2.5)  # 设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors(100)  # 设置样本点可搜索的邻域个数
    gp3.setMaximumSurfaceAngle(np.pi / 4)  # 设置某点法线方向偏离样本点法线的最大角度45
    gp3.setMinimumAngle(np.pi / 8)  # 设置三角化后得到的三角形内角的最小的角度为10
    gp3.setMaximumAngle(2 * np.pi / 3)  # 设置三角化后得到的三角形内角的最大角度为120
    gp3.setNormalConsistency(True)  # 设置该参数保证法线朝向一致

    # 计算结果
    gp3.setInputCloud(cloud_with_normals)
    gp3.setSearchMethod(tree)
    gp3.reconstruct(triangles)

    # 获取顶点信息
    triangles = triangles.polygons

    # 算法参数
    support_radius = 0.0285  # 局部表面裁剪的支持半径
    number_of_partition_bins = 5  # 用于形成分布矩阵的分区数和
    number_of_rotations = 3  # 用于形成分布矩阵的旋转数,影响描述符的长度

    tree = pcl.search.KdTree.PointXYZ()
    tree.setInputCloud(cloud)

    feature_estimator = pcl.features.ROPSEstimation.PointXYZ_Histogram_135()
    feature_estimator.setSearchMethod(tree)
    feature_estimator.setSearchSurface(cloud)
    feature_estimator.setInputCloud(cloud)
    feature_estimator.setIndices(indices)
    feature_estimator.setTriangles(triangles)
    feature_estimator.setRadiusSearch(support_radius)
    feature_estimator.setNumberOfPartitionBins(number_of_partition_bins)
    feature_estimator.setNumberOfRotations(number_of_rotations)
    feature_estimator.setSupportRadius(support_radius)

    # histograms = pcl.features.Feature.PointXYZ_Histogram_135()
    # feature_estimator.compute(histograms)
    # plt.plot(histograms)