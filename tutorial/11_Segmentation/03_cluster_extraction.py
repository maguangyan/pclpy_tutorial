# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/6
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 03_cluster_extraction.py

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
    reader = pcl.io.PCDReader()
    cloud = pcl.PointCloud.PointXYZ()
    cloud_f = pcl.PointCloud.PointXYZ()
    reader.read('../../data/table_scene_lms400.pcd', cloud)
    print('PointCloud before filtering has: ', cloud.size(), ' data points.')

    # 下采样 使用leaf size为1cm
    vg = pcl.filters.VoxelGrid.PointXYZ()
    cloud_filtered = pcl.PointCloud.PointXYZ()
    vg.setInputCloud(cloud)
    vg.setLeafSize(0.01, 0.01, 0.01)
    vg.filter(cloud_filtered)
    print('PointCloud after filtering has: ', cloud_filtered.size(), ' data points.')

    # 平面分割
    seg = pcl.segmentation.SACSegmentation.PointXYZ()
    inliers = pcl.PointIndices()
    coefficients = pcl.ModelCoefficients()
    cloud_plane = pcl.PointCloud.PointXYZ()
    writer = pcl.io.PCDWriter()
    seg.setOptimizeCoefficients(True)
    seg.setModelType(0)
    seg.setMethodType(0)
    seg.setMaxIterations(100)
    seg.setDistanceThreshold(0.02)

    nr_points = cloud_filtered.size()
    while cloud_filtered.size() > 0.3 * nr_points:
        # 在剩余的点云中分割出最大的平面成分
        seg.setInputCloud(cloud_filtered)
        seg.segment(inliers, coefficients)
        if len(inliers.indices) == 0:
            print('Could not estimate a planar model for the given dataset.')
            break

        # 提取平面内点
        extract = pcl.filters.ExtractIndices.PointXYZ()
        extract.setInputCloud(cloud_filtered)
        extract.setIndices(inliers)
        extract.setNegative(False)

        extract.filter(cloud_plane)
        # 可视化提取出来的平面
        compareCloudShow(cloud_filtered, cloud_plane)
        print('PointCloud representing the planar component: ', cloud_plane.size(), ' data points.')

        # 去除上面提取到的平面内点，得到剩余点
        extract.setNegative(True)
        extract.filter(cloud_f)
        cloud_filtered = cloud_f

    writer.write('cloud_cluster_source.pcd', cloud_filtered, False)
    # 创建KdTree
    tree = pcl.search.KdTree.PointXYZ()
    tree.setInputCloud(cloud_filtered)

    cluster_indices = pcl.vectors.PointIndices()
    ec = pcl.segmentation.EuclideanClusterExtraction.PointXYZ()
    ec.setClusterTolerance(0.02)  # 2cm
    ec.setMinClusterSize(100)
    ec.setMaxClusterSize(25000)
    ec.setSearchMethod(tree)
    ec.setInputCloud(cloud_filtered)
    ec.extract(cluster_indices)

    j = 0
    for it in cluster_indices:
        cloud_cluster = pcl.PointCloud.PointXYZ()
        for pit in it.indices:
            cloud_cluster.push_back(cloud_filtered.at(pit))
        cloud_cluster.width = cloud_cluster.size()
        cloud_cluster.height = 1
        cloud_cluster.is_dense = True

        print('PointCloud representing the Cluster: ', cloud_cluster.size(), ' data points.')

        ss = 'cloud_cluster_' + str(j) + '.pcd'
        writer.write(ss, cloud_cluster, False)
        j = j + 1
