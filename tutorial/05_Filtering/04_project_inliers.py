# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/22
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 04_project_inliers.py

import pclpy
from pclpy import pcl
import numpy as np


def filterCloudShow(cloud, cloud_filtered):
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
    # 生成点云数据
    cloud_size = 1000
    a = np.random.ranf(cloud_size * 3).reshape(-1, 3) * 1024
    cloud = pcl.PointCloud.PointXYZ.from_array(a)
    cloud_projected = pcl.PointCloud.PointXYZ()

    print('Cloud before projection:')
    for point in cloud.points:
        print(point.x, ' ',
              point.y, ' ',
              point.z, ' ')

    # 创建一个平面，系数为：x=y=0, z=1
    coeffs = pcl.ModelCoefficients()
    coeffs.values.append(0.0)
    coeffs.values.append(0.0)
    coeffs.values.append(1.0)
    coeffs.values.append(0.0)

    # 创建滤波器
    proj = pcl.filters.ProjectInliers.PointXYZ()
    proj.setModelType(0)
    proj.setInputCloud(cloud)
    proj.setModelCoefficients(coeffs)
    proj.filter(cloud_projected)

    print('Cloud after  projection:')
    for point in cloud_projected.points:
        print(point.x, ' ',
              point.y, ' ',
              point.z, ' ')
    # 主循环
    filterCloudShow(cloud, cloud_projected)

