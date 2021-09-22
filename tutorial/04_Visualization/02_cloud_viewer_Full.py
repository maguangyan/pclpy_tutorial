# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_cloud_viewer.py

import pclpy
from pclpy import pcl



def viewerOneOff(viewer):
    viewer.setBackgroundColor(1.0, 0.5, 1.0)
    o = pcl.point_types.PointXYZ()
    o.x = 1
    o.y = 0
    o.z = 0
    viewer.addSphere(o, 0.25, "sphere", 0)
    print('i only run once')


def viewerPsycho(viewer):
    i = 0
    ss = "Once per viewer loop: " + str(i)
    viewer.removeShape("text", 0)
    viewer.addText(ss, 200, 300, "text", 0)


if __name__ == '__main__':
    # 加载点云
    cloud = pcl.PointCloud.PointXYZRGB()
    reader = pcl.io.PCDReader()
    reader.read("../../data/bunny_color.pcd", cloud)

    # 可视化
    viewer = pcl.visualization.CloudViewer('Cloud Viewer')
    # 阻塞，直到点云真正渲染
    viewer.showCloud(cloud)
    # 使用下面的函数来访问底层更高级/强大的功能
    # 使用PCLVisualizer

    # # 这只会被调用一次
    viewer.runOnVisualizationThreadOnce(viewerOneOff)

    # 每次可视化迭代都会调用这个函数一次
    viewer.runOnVisualizationThread(viewerPsycho)
    while not viewer.wasStopped(10):
        pass
