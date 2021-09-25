# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/25
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 08_moment_of_inertia.py

import pclpy
from pclpy import pcl
import numpy as np
import sys
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZ()
    reader = pcl.io.PCDReader()
    reader.read("../../data/min_cut_segmentation_tutorial.pcd", cloud)

    print(cloud.size())

    feature_extractor = pcl.features.MomentOfInertiaEstimation.PointXYZ()
    feature_extractor.setInputCloud(cloud)
    feature_extractor.compute()

    moment_of_inertia = pcl.vectors.Float()
    eccentricity = pcl.vectors.Float()

    min_point_AABB = pcl.point_types.PointXYZ()
    max_point_AABB = pcl.point_types.PointXYZ()
    min_point_OBB = pcl.point_types.PointXYZ()
    max_point_OBB = pcl.point_types.PointXYZ()
    position_OBB = pcl.point_types.PointXYZ()

    rotational_matrix_OBB = np.zeros((3, 3), np.float)
    major_value, middle_value, minor_value = 0.0, 0.0, 0.0
    major_vector = np.zeros((3, 1), np.float)
    middle_vector = np.zeros((3, 1), np.float)
    minor_vector = np.zeros((3, 1), np.float)
    mass_center = np.zeros((3, 1), np.float)

    feature_extractor.getMomentOfInertia(moment_of_inertia)
    feature_extractor.getEccentricity(eccentricity)
    feature_extractor.getAABB(min_point_AABB, max_point_AABB)
    feature_extractor.getEigenValues(major_value, middle_value, minor_value)    # 后面这三行似乎都不怎么管用，使用numpy数组直接作为参数的函数似乎都有这个问题。
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector)
    feature_extractor.getMassCenter(mass_center)



    viewer = pcl.visualization.PCLVisualizer('3D Viewer')
    viewer.setBackgroundColor(0.0, 0.0, 0.0)
    viewer.addCoordinateSystem(1.0)
    viewer.initCameraParameters()
    viewer.addPointCloud(cloud, 'simple cloud')
    viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y,
                   min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, 'AABB')
    viewer.setShapeRenderingProperties(5, 1, 'AABB')  # 点云用线框表示

    position = np.array([[position_OBB.x], [position_OBB.y], [position_OBB.z]],
                        np.float)
    quat = pclpy.pcl.vectors.Quaternionf()
    viewer.addCube(position, quat, max_point_OBB.x - min_point_OBB.x,
                   max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB")
    viewer.setShapeRenderingProperties(5, 1, "OBB")

    center = pcl.point_types.PointXYZ()
    center.x, center.y, center.z = mass_center[0], mass_center[1], mass_center[2]

    x_axis = pcl.point_types.PointXYZ()
    x_axis.x, x_axis.y, x_axis.z = major_vector[0] + mass_center[0],\
                                   major_vector[1] + mass_center[1],\
                                   major_vector[2] + mass_center[2]

    y_axis = pcl.point_types.PointXYZ()
    y_axis.x, y_axis.y, y_axis.z = middle_vector[0] + mass_center[0],\
                                   middle_vector[1] + mass_center[1],\
                                   middle_vector[2] + mass_center[2]

    z_axis = pcl.point_types.PointXYZ()
    z_axis.x, z_axis.y, z_axis.z = minor_vector[0] + mass_center[0], \
                                   minor_vector[1] + mass_center[1], \
                                   minor_vector[2] + mass_center[2]
    viewer.addLine(center, x_axis, 1.0, 0.0, 0.0,  "major eigen vector")
    viewer.addLine(center, y_axis, 0.0, 1.0, 0.0, "middle eigen vector")
    viewer.addLine(center, z_axis, 0.0, 0.0, 1.0, "minor eigen vector")

    while not viewer.wasStopped():
        viewer.spinOnce(10)
