# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/9
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 04_bspline_fitting.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

def PointCloud2Vector3d(cloud, data):
    pass

def visualizeCurve(curve, surface, viewer):
    pass


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: pcl_example_nurbs_fitting_surface pcd<PointXYZ>-in-file 3dm-out-file')
        exit(0)
    pcd_file = sys.argv[1]
    file_3dm = sys.argv[2]

    viewer = pcl.visualization.PCLVisualizer()
    viewer.setSize(800, 600)

    # 加载点云数据
    cloud = pcl.PointCloud.PointXYZ()
    if pcl.io.loadPCDFile(pcd_file, cloud) < 0:
        print('  PCD file not found.')
        exit(-1)

    blue = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 0.0, 255.0)
    viewer.addPointCloud(cloud, blue, "cloud")

    viewer.setPointCloudRenderingProperties(0, 3, "cloud")
    while not viewer.wasStopped():
        viewer.spinOnce(10)
