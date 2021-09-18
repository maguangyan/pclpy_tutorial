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

    # 可视化法线
    viewer = pcl.visualization.PCLVisualizer("viewer")
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    viewer.addPointCloud(cloud, "sample cloud")
    viewer.addPointCloudNormals(cloud, normals)

    while not viewer.wasStopped():
        viewer.spinOnce(10)
