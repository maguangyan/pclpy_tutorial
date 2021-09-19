# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/18
# @Author : yan
# @Email : 1792659158@qq.com
# @File : normal_estimation_using_integral_images.py

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
    normals = pcl.PointCloud.Normal()
    ne = pcl.features.IntegralImageNormalEstimation.PointXYZ_Normal()
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT)
    ne.setMaxDepthChangeFactor(0.02)
    ne.setNormalSmoothingSize(10.0)
    ne.setInputCloud(cloud)
    ne.compute(normals)

    # # 可视化
    # viewer = pcl.visualization.PCLVisualizer("viewer")
    # viewer.setBackgroundColor(0.0, 0.0, 0.0)
    # viewer.addPointCloud(cloud, "sample cloud")  # 显示点云
    # # viewer.addPointCloudNormals(cloud, normals)  # 显示点云和法线 该函数bug未修复~
    #
    # while not viewer.wasStopped():
    #     viewer.spinOnce(10)

    # 使用pyvista可视化
    p = pv.Plotter()
    cloud = pv.wrap(cloud.xyz)
    p.add_mesh(cloud, point_size=1, color='g')
    p.camera_position = 'iso'
    p.enable_parallel_projection()
    p.show_axes()
    p.show()
