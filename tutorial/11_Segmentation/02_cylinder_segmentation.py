# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/5
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 02_cylinder_segmentation.py

import pclpy
from pclpy import pcl
import numpy as np

if __name__ == '__main__':
    # 所有需要的类实例化
    reader = pcl.io.PCDReader()
    ps = pcl.filters.PassThrough.PointXYZRGBA()
    ne = pcl.features.NormalEstimation.PointXYZRGBA_Normal()
    seg = pcl.segmentation.SACSegmentationFromNormals.PointXYZRGBA_Normal()
    writer = pcl.io.PCDWriter()
    extract = pcl.filters.ExtractIndices.PointXYZRGBA()
    extract_normals = pcl.filters.ExtractIndices.Normal()
    tree = pcl.search.KdTree.PointXYZRGBA()

    #  Datasets
    cloud = pcl.PointCloud.PointXYZRGBA()
    cloud_filtered = pcl.PointCloud.PointXYZRGBA()
    cloud_normals = pcl.PointCloud.Normal()
    cloud_filtered2 = pcl.PointCloud.PointXYZRGBA()
    cloud_normals2 = pcl.PointCloud.Normal()
    coefficients_plane = pcl.ModelCoefficients()
    coefficients_cylinder = pcl.ModelCoefficients()
    inliers_plane = pcl.PointIndices()
    inliers_cylinder = pcl.PointIndices()

    # 加载点云
    reader.read("../../data/table_scene_mug_stereo_textured.pcd", cloud)
    print('PointCloud has: ', cloud.size(), 'data points.')

    # 使用带通滤波器去除无效点（外点和无穷点）
    ps.setInputCloud(cloud)
    ps.setFilterFieldName('z')
    ps.setFilterLimits(0, 1.5)
    ps.filter(cloud_filtered)
    print('PointCloud after filtering has: ', cloud_filtered.size(), 'data points.')

    # 估计法线
    ne.setInputCloud(cloud_filtered)
    ne.setSearchMethod(tree)
    ne.setKSearch(50)
    ne.compute(cloud_normals)

    # 实例化平面分割类并设置参数
    seg.setOptimizeCoefficients(True)
    seg.setModelType(0)  # 0为pcl::SACMODEL_PLANE
    seg.setMethodType(0)  # 0为pcl::SAC_RANSAC
    seg.setNormalDistanceWeight(0.1)
    seg.setMaxIterations(100)
    seg.setDistanceThreshold(0.03)
    seg.setInputCloud(cloud_filtered)
    seg.setInputNormals(cloud_normals)
    # 获取平面内点和系数
    seg.segment(inliers_plane, coefficients_plane)
    print('Plane coefficients:', coefficients_plane.values)

    # 提取平面内点
    extract.setInputCloud(cloud_filtered)
    extract.setIndices(inliers_plane)
    extract.setNegative(False)
    # 将内点存入磁盘
    cloud_plane = pcl.PointCloud.PointXYZRGBA()
    extract.filter(cloud_plane)
    print('PointCloud representing the planar component: ', cloud_plane.size(), ' data points.')
    writer.write('table_scene_mug_stereo_textured_plane.pcd', cloud_plane, False)

    # 去除平面内点，提取剩余点
    extract.setNegative(True)
    extract.filter(cloud_filtered2)

    extract_normals.setInputCloud(cloud_normals)
    extract_normals.setNegative(True)
    extract_normals.setIndices(inliers_plane)
    extract_normals.filter(cloud_normals2)

    # 实例化圆柱分割类并设置参数
    seg.setOptimizeCoefficients(True)
    seg.setModelType(5)     # 5代表SACMODEL_CYLINDER
    seg.setMethodType(0)    # 0代表SAC_RANSAC
    seg.setNormalDistanceWeight(0.1)
    seg.setMaxIterations(10000)
    seg.setDistanceThreshold(0.05)
    seg.setRadiusLimits(0, 0.1)
    seg.setInputCloud(cloud_filtered2)
    seg.setInputNormals(cloud_normals2)

    # 获取椭圆内点和参数
    seg.segment(inliers_cylinder, coefficients_cylinder)
    print('Cylinder coefficients: ', coefficients_cylinder.values)

    # 将椭圆点云写入磁盘
    extract.setInputCloud(cloud_filtered2)
    extract.setIndices(inliers_cylinder)
    extract.setNegative(False)
    cloud_cylinder = pcl.PointCloud.PointXYZRGBA()
    extract.filter(cloud_cylinder)

    if cloud_cylinder.size() == 0:
        print('Can not find the cylindrical component.')
    else:
        print('PointCloud representing the cylindrical component: ', cloud_cylinder.size())
        writer.write('table_scene_mug_stereo_textured_cylinder.pcd', cloud_cylinder, False)

    # 可视化
    viewer = pcl.visualization.PCLVisualizer("viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v0)
    viewer.addText("plane", 10, 10, "v1 text", v0)
    viewer.addPointCloud(cloud_plane, "plane cloud", v0)

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0.3, 0.3, 0.3, v1)
    viewer.addText("cylinder", 10, 10, "v2 text", v1)
    viewer.addPointCloud(cloud_cylinder, "cylinder cloud", v1)

    viewer.setPointCloudRenderingProperties(0, 1, "plane cloud", v0)
    viewer.setPointCloudRenderingProperties(0, 1, "cylinder cloud", v1)
    viewer.addCoordinateSystem(1.0)

    while not viewer.wasStopped():
        viewer.spinOnce(10)