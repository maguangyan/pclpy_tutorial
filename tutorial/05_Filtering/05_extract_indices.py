# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 05_extract_indices.py

import pclpy
from pclpy import pcl
import numpy as np


def compareCloudShow(cloud, cloud_filtered):
    # Open 3D viewer and add point cloud and normals
    viewer = pcl.visualization.PCLVisualizer("viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v0)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud, single_color, "sample cloud1", v0)

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v1)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud_filtered, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud_filtered, single_color, "sample cloud2", v1)

    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud1", v0)
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud2", v1)
    viewer.addCoordinateSystem(1.0)
    while not viewer.wasStopped():
        viewer.spinOnce(10)


if __name__ == '__main__':
    # 读取点云数据
    cloud = pcl.PointCloud.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    cloud_p = pcl.PointCloud.PointXYZ()
    cloud_f = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read('../../data/table_scene_lms400.pcd', cloud)
    print("PointCloud before filtering: ",
          cloud.width * cloud.height, " data points ")

    # 创建sor滤波器
    sor = pcl.filters.StatisticalOutlierRemoval.PointXYZ()
    sor.setInputCloud(cloud)
    sor.setMeanK(50)
    sor.setStddevMulThresh(1.0)
    sor.filter(cloud_filtered)
    # 可视化滤波效果
    compareCloudShow(cloud, cloud_filtered)

    coeffs = pcl.ModelCoefficients()
    inliers = pcl.PointIndices()
    # 创建分割object
    seg = pcl.segmentation.SACSegmentation.PointXYZ()
    # 可选项
    seg.setOptimizeCoefficients(True)
    # 设置
    seg.setModelType(0)  # 0为pcl::SACMODEL_PLANE
    seg.setMethodType(0)  # 0为pcl::SAC_RANSAC
    seg.setMaxIterations(1000)
    seg.setDistanceThreshold(0.01)

    # 创建滤波object
    extract = pcl.filters.ExtractIndices.PointXYZ()
    nr_points = cloud_filtered.size()
    while cloud_filtered.size() > nr_points * 0.3:
        # 从保留的点云中分割最大的平面成分
        seg.setInputCloud(cloud_filtered)
        seg.segment(inliers, coeffs)
        if len(inliers.indices) == 0:
            print('Could not estimate a planar model for the given dataset.')
            break

        # 提取内点(平面成分)
        extract.setInputCloud(cloud_filtered)
        extract.setIndices(inliers)
        extract.setNegative(False)
        extract.filter(cloud_p)
        # 可视化提取出来的平面
        compareCloudShow(cloud_filtered, cloud_p)
        print('PointCloud representing the planar component: ',
              cloud_p.width * cloud_p.height, 'data points.')

        # 再次滤波，提取外点（非平面成分）
        extract.setNegative(True)
        extract.filter(cloud_f)
        cloud_filtered.swap(cloud_f)  # 等价于cloud_filtered = cloud_f


