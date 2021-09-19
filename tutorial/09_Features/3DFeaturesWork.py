# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/18
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 3DFeaturesWork.py

import os
import numpy as np
import pytest

import pclpy
from pclpy import pcl
import pyvista as pv

if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/table_scene_mug_stereo_textured.pcd", cloud)
    # 估计法线
    # 为输入数据集中的所有点估计一组表面法线
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)
    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)

    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.003)
    ne.compute(cloud_normals)
    print(cloud_normals.size())

    # 为输入数据集中的点子集估计一组表面法线。
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    ne.setInputCloud(cloud)
    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)
    ind = pcl.PointIndices()
    [ind.indices.append(i) for i in range(0, cloud.size() // 2)]
    ne.setIndices(ind)
    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.003)
    ne.compute(cloud_normals)
    print(cloud_normals.size())

    # 为输入数据集中的所有点估计一组表面法线，但使用另一个数据集估计它们的最近邻
    ne = pcl.features.NormalEstimation.PointXYZ_Normal()
    cloud_downsampled = pcl.PointCloud.PointXYZ()  # 获取一个降采样的点云 方法比较多，这里使用voxelized方法
    vox = pcl.filters.VoxelGrid.PointXYZ()
    vox.setInputCloud(cloud)
    vox.setLeafSize(0.005, 0.005, 0.005)
    vox.filter(cloud_downsampled)
    ne.setInputCloud(cloud_downsampled)
    ne.setSearchSurface(cloud)
    tree = pcl.search.KdTree.PointXYZ()
    ne.setSearchMethod(tree)
    cloud_normals = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.003)
    ne.compute(cloud_normals)
    print(cloud_normals.size())

    # 使用pyvista可视化
    p = pv.Plotter(shape=(1, 2))
    p.subplot(0, 0)
    cloud = pv.wrap(cloud.xyz)
    p.add_mesh(cloud, point_size=1, color='g')
    p.camera_position = 'iso'
    p.enable_parallel_projection()
    p.show_axes()
    p.link_views()

    p.subplot(0, 1)
    cloud_downsampled = pv.wrap(cloud_downsampled.xyz)
    p.add_mesh(cloud_downsampled, point_size=1, color='g')
    p.camera_position = 'iso'
    p.enable_parallel_projection()
    p.show_axes()

    p.show()
