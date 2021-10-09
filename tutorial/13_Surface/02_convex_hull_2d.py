# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/9
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 02_convex_hull_2d.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 加载点云数据
    cloud = pcl.PointCloud.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    cloud_projected = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile('../../data/table_scene_mug_stereo_textured.pcd', cloud) < 0:
        print('Error loading model cloud.')
        exit(-1)
    # 滤波
    ps = pcl.filters.PassThrough.PointXYZ()
    ps.setInputCloud(cloud)
    ps.setFilterFieldName('z')
    ps.setFilterLimits(0, 1.0)
    ps.filter(cloud_filtered)
    print('PointCloud after filtering has: ', cloud_filtered.size(), ' data points.')

    coefficients = pcl.ModelCoefficients()
    inliers = pcl.PointIndices()
    # 实例化分割类
    seg = pcl.segmentation.SACSegmentation.PointXYZ()
    # 可选项
    seg.setOptimizeCoefficients(True)
    # 必需设置的参数
    seg.setInputCloud(cloud_filtered)
    seg.setModelType(0)  # 0为pcl::SACMODEL_PLANE
    seg.setMethodType(0)  # 0为pcl::SAC_RANSAC
    seg.setDistanceThreshold(0.01)

    seg.segment(inliers, coefficients)
    print('PointCloud after segmentation has: ',
          len(inliers.indices), ' inliers.')

    # 投影模型内点
    proj = pcl.filters.ProjectInliers.PointXYZ()
    proj.setModelType(0)  # 0为pcl::SACMODEL_PLANE
    proj.setIndices(inliers)
    proj.setInputCloud(cloud_filtered)
    proj.setModelCoefficients(coefficients)
    proj.filter(cloud_projected)
    print('PointCloud after projection has: ',
          cloud_projected.size(), ' data points.')

    # 创建一个凹壳表示投影内点
    cloud_Concavehull = pcl.PointCloud.PointXYZ()
    Concavehull = pcl.surface.ConcaveHull.PointXYZ()
    Concavehull.setInputCloud(cloud_projected)
    Concavehull.setAlpha(0.1)
    Concavehull.reconstruct(cloud_Concavehull)

    print('Concave hull has: ', cloud_Concavehull.size(), ' data points')

    # 创建一个凸壳表示投影内点
    cloud_Convexhull = pcl.PointCloud.PointXYZ()
    Convexhull = pcl.surface.ConvexHull.PointXYZ()
    Convexhull.setInputCloud(cloud_projected)
    Convexhull.reconstruct(cloud_Convexhull)
    print('Convex hull has: ', cloud_Convexhull.size(), ' data points')

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)

    white = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_projected, 255.0, 255.0, 255.0)
    viewer.addPointCloud(cloud_projected, white, "projected cloud")

    green = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_Concavehull, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud_Concavehull, green, "Concavehull cloud")

    blue = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_Convexhull, 0.0, 0.0, 255.0)
    viewer.addPointCloud(cloud_Convexhull, blue, "Convexhull cloud")

    viewer.setPointCloudRenderingProperties(0, 3, "projected cloud")
    viewer.setPointCloudRenderingProperties(0, 3, "Concavehull cloud")
    viewer.setPointCloudRenderingProperties(1, 0.8, 'Concavehull cloud')
    viewer.setPointCloudRenderingProperties(0, 3, "Convexhull cloud")
    viewer.setPointCloudRenderingProperties(1, 0.8, 'Convexhull cloud')
    while not viewer.wasStopped():
        viewer.spinOnce(10)
