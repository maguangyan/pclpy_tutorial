# -*- coding: utf-8 -*-
# @Time : DATE:2021/9/20
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_cloud_viewer.py

import pclpy
from pclpy import pcl
import numpy as np
import sys


def printUsage(progName):
    # 帮助函数
    print('Usage:', progName, ' [options]\n',
          "Options:\n",
          "-------------------------------------------\n",
          "-h           this help\n",
          "-s           Simple visualisation example\n",
          "-r           RGB colour visualisation example\n",
          "-c           Custom colour visualisation example\n"
          "-n           Normals visualisation example\n",
          "-a           Shapes visualisation example\n",
          "-v           Viewports example\n",
          "-i           Interaction Customization example\n",
          )


def simpleVis(cloud):
    # Open 3D viewer and add point cloud
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    viewer.addPointCloud(cloud, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()
    return viewer


def rgbVis(cloud):
    # Open 3D viewer and add point cloud
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(cloud)

    viewer.addPointCloud(cloud, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()
    return viewer


def customColourVis(cloud):
    # Open 3D viewer and add point cloud
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    single_color  = pcl.visualization.PointCloudColorHandlerCustom.PointXYZ(cloud, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud, single_color , "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 3, "sample cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()
    return viewer


def normalsVis(cloud, normals):
    # Open 3D viewer and add cloud and normals
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(cloud)
    viewer.addPointCloud(cloud, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
    viewer.addPointCloudNormals(cloud, normals, 10, 0.05, 'normals')  # 经过测试，法线显示报错：no override found for "VtkActor"
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()
    return viewer


def shapesVis(cloud):
    # Open 3D viewer and add point cloud
    viewer = pcl.visualization.PCLVisualizer("3D viewer")
    viewer.setBackgroundColor(0, 0, 0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(cloud)
    viewer.addPointCloud(cloud, rgb, "sample cloud")
    viewer.setPointCloudRenderingProperties(0, 1, "sample cloud")
    viewer.addCoordinateSystem(1)
    viewer.initCameraParameters()
    # 在点云上添加一些直线、椭圆 经过测试，除了addSphere报错：no override found for "VTKPolyDataMapper"，其他都还OK
    xyzCloud = pcl.PointCloud.PointXYZ().from_array(cloud.xyz)
    viewer.addLine(xyzCloud.points[0], xyzCloud.points[cloud.size() - 1], 'line')  # 起点、终点
    # viewer.addSphere(xyzCloud.points[0], 0.2, 0.5, 0.5, 0.0, 'sphere')  # 中心，半径、RGB
    # 在其他位置添加一些平面、锥
    coeffs = pcl.ModelCoefficients()
    coeffs.values.append(0.0)
    coeffs.values.append(0.0)
    coeffs.values.append(1.0)
    coeffs.values.append(0.0)
    viewer.addPlane(coeffs, 'plane')
    coeffs = pcl.ModelCoefficients()
    coeffs.values.append(0.3)
    coeffs.values.append(0.3)
    coeffs.values.append(0.0)
    coeffs.values.append(0.0)
    coeffs.values.append(1.0)
    coeffs.values.append(0.0)
    coeffs.values.append(5.0)
    viewer.addCone(coeffs, 'cone')
    return viewer


def viewportsVis(cloud, normals1, normals2):
    # Open 3D viewer and add point cloud and normals
    viewer = pcl.visualization.PCLVisualizer("viewer")
    v0 = 1
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v0)
    viewer.setBackgroundColor(0.0, 0.0, 0.0, v0)
    viewer.addText("Radius: 0.05", 10, 10, "v1 text", v0)
    rgb = pcl.visualization.PointCloudColorHandlerRGBField.PointXYZRGB(cloud)
    viewer.addPointCloud(cloud, rgb, "sample cloud1", v0)

    v1 = 2
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v1)
    viewer.setBackgroundColor(0.3, 0.3, 0.3, v1)
    viewer.addText("Radius: 0.1", 10, 10, "v2 text", v1)
    single_color = pcl.visualization.PointCloudColorHandlerCustom.PointXYZRGB(cloud, 0.0, 255.0, 0.0)
    viewer.addPointCloud(cloud, single_color, "sample cloud2", v1)

    viewer.setPointCloudRenderingProperties(0, 3, "sample cloud1", v0)
    viewer.setPointCloudRenderingProperties(0, 3, "sample cloud2", v1)
    viewer.addCoordinateSystem(1.0)

    # viewer.addPointCloudNormals(cloud, cloud_normals1, 10, 0.05, 'normals1', v0)  # 法线无法显示，报错：no override found for "VtkActor"
    # viewer.addPointCloudNormals(cloud, cloud_normals2, 10, 0.05, 'normals2', v1)
    return viewer


def keyboardEventOccurred(event):
    if event.getKeySym() == 'r' and event.keyDown():
        print('r was pressed => removing all text')
        for i in range(text_id[0] + 1):
            str = 'text{:2d}'.format(i)
            viewer.removeShape(str)
        text_id[0] = 0


def mouseEventOccurred(event):
    if event.getButton() == pcl.visualization.MouseEvent.LeftButton and \
            event.getType() == pcl.visualization.MouseEvent.MouseButtonRelease:
        print("Left mouse button released at position (", event.getX(), ", ", event.getY(), ")")
        str = 'text{:2d}'.format(text_id[0])
        text_id[0] += 1
        viewer.addText("welcom to star my github", event.getX(), event.getY(), str)


def interactionCustomizationVis():
    viewer = pcl.visualization.PCLVisualizer()
    viewer.setBackgroundColor(0, 0, 0)
    viewer.addCoordinateSystem(1.0)
    global text_id
    text_id = [0]
    viewer.registerKeyboardCallback(keyboardEventOccurred)
    viewer.registerMouseCallback(mouseEventOccurred)
    return viewer


if __name__ == '__main__':
    # 生成点云
    basic_cloud = pcl.PointCloud.PointXYZ()
    point_cloud = pcl.PointCloud.PointXYZRGB()
    print('Generating example point clouds.')
    r, g, b = 255, 15, 15
    for z in np.linspace(-1, 1, 40):
        for angle in np.linspace(0, 360, 72):
            basic_point = pcl.point_types.PointXYZ()
            basic_point.x = 0.5 * np.cos(np.deg2rad(angle))
            basic_point.y = np.sin(np.deg2rad(angle))
            basic_point.z = z
            basic_cloud.push_back(basic_point)

            point = pcl.point_types.PointXYZRGB()
            point.x = basic_point.x
            point.y = basic_point.y
            point.z = basic_point.z
            point.r = np.uint8(r)
            point.g = np.uint8(g)
            point.b = np.uint8(b)

            point_cloud.push_back(point)
            if z < 0.0:
                r -= 12
                g += 12
            else:
                g -= 12
                b += 12
    basic_cloud.width = basic_cloud.size()
    basic_cloud.height = 1
    point_cloud.width = point_cloud.size()
    point_cloud.height = 1

    # 使用0.05的搜索半径计算表面法线
    ne = pcl.features.NormalEstimation.PointXYZRGB_Normal()
    ne.setInputCloud(point_cloud)
    tree = pcl.search.KdTree.PointXYZRGB()
    ne.setSearchMethod(tree)
    cloud_normals1 = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.05)
    ne.compute(cloud_normals1)

    # 使用0.01的搜索半径计算表面法线
    cloud_normals2 = pcl.PointCloud.Normal()
    ne.setRadiusSearch(0.1)
    ne.compute(cloud_normals2)

    # 可视化
    viewer = None
    if len(sys.argv) == 1:
        printUsage(sys.argv[0])
        sys.exit()
    if sys.argv[1] == '-h':
        printUsage(sys.argv[0])
        sys.exit()
    if sys.argv[1] == '-s':
        print("Simple visualisation example")
        viewer = simpleVis(basic_cloud)
    elif sys.argv[1] == '-r':
        print("RGB colour visualisation example")
        viewer = rgbVis(point_cloud)
    elif sys.argv[1] == '-c':
        print("Custom colour visualisation example")
        viewer = customColourVis(basic_cloud)
    elif sys.argv[1] == '-n':
        print("Normals visualisation example")
        viewer = normalsVis(point_cloud, cloud_normals1)
    elif sys.argv[1] == '-a':
        print("Shapes visualisation example")
        viewer = shapesVis(point_cloud)
    elif sys.argv[1] == '-v':
        print("Viewports example")
        viewer = viewportsVis(point_cloud, cloud_normals1, cloud_normals2)
    elif sys.argv[1] == '-i':
        print("Interaction Customization example")
        viewer = interactionCustomizationVis()

    # 主循环
    while not viewer.wasStopped():
        viewer.spinOnce(10)
