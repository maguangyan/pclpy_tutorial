# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/5
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_planar_segmentation.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 生成点云
    cloud_size = 15
    a = np.zeros((cloud_size, 3))
    cloud = pcl.PointCloud.PointXYZ().from_array(a)
    for point in cloud.points:
        point.x = np.random.ranf() * 1024
        point.y = np.random.ranf() * 1024
        point.z = 1.0
    # 设置一些外点
    cloud.points[0].z = 4.0
    cloud.points[3].z = -4.0
    cloud.points[6].z = -8.0

    print('Point cloud data:', cloud.size(), 'points')
    for point in cloud.points:
        print(point.x, ' ', point.y, ' ', point.z)

    coefficients = pcl.ModelCoefficients()
    inliers = pcl.PointIndices()

    # 分割类实例化
    seg = pcl.segmentation.SACSegmentation.PointXYZ()
    # 可选项
    seg.setOptimizeCoefficients(True)
    # 必须要指定的参数
    seg.setModelType(0)  # 0为pcl::SACMODEL_PLANE
    seg.setMethodType(0)  # 0为pcl::SAC_RANSAC
    seg.setDistanceThreshold(0.01)

    seg.setInputCloud(cloud)
    seg.segment(inliers, coefficients)

    if len(inliers.indices) == 0:
        print('Could not estimate a planar model for the given dataset.')
        exit()

    print('Model coefficients:',
          coefficients.values[0], ' ',
          coefficients.values[1], ' ',
          coefficients.values[2], ' ',
          coefficients.values[2])

    print('Model inliers: ', len(inliers.indices))
    for idx in inliers.indices:
        print(idx, '    ',
              cloud.points[idx].x, ' ',
              cloud.points[idx].y, ' ',
              cloud.points[idx].z)

    # 可视化内点和外点
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    # 内点
    inliers_cloud = pcl.PointCloud.PointXYZ(cloud, inliers.indices)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(inliers_cloud, 0.0, 255.0, 0.0)  # 内点设为绿色
    viewer.addPointCloud(inliers_cloud, single_color, "inliers cloud")
    # 外点
    outliers_cloud = pcl.PointCloud.PointXYZ()
    extract = pcl.filters.ExtractIndices.PointXYZ()
    extract.setInputCloud(cloud)
    extract.setIndices(inliers)
    extract.setNegative(True)  # 提取出来外点点云
    extract.filter(outliers_cloud)

    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(outliers_cloud, 255.0, 0.0, 0.0)  # 外点设为红色
    viewer.addPointCloud(outliers_cloud, single_color, "outliers cloud")

    viewer.setPointCloudRenderingProperties(0, 3, "inliers cloud")
    viewer.setPointCloudRenderingProperties(0, 3, "outliers cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()

    while not viewer.wasStopped():
        viewer.spinOnce(10)
