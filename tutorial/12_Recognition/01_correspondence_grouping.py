# -*- coding: utf-8 -*-
# @Time : DATE:2021/10/8
# @Author : yan
# @Email : 1792659158@qq.com
# @File : 01_correspondence_grouping.py

import pclpy
from pclpy import pcl
import numpy as np
import sys

# 算法参数
params = {'show_keypoints_': False,
         'show_correspondences_': False,
         'use_cloud_resolution_': False,
         'use_hough_': True,
         'model_ss_': 0.01,
         'scene_ss_': 0.03,
         'rf_rad_': 0.015,
         'descr_rad_': 0.02,
         'cg_size_': 0.01,
         'cg_thresh_': 5.0}

filenames_params = {'model_filename_':'', 'scene_filename_':''}

def parse_argument(argv, param,  variable):
    """
    根据命令行参数对变量进行赋值
    :param argv: 命令行参数列表
    :param param: 需要赋值的命令
    :param variable: 命令对应的变量名
    :return:
    """
    if param in argv:
        index = argv.index(param) + 1
        params[variable] = float(sys.argv[index])


def showHelp(filename):
    print(
        "***************************************************************************\n",
        "*                                                                         *\n",
        "*             Correspondence Grouping Tutorial - Usage Guide              *\n",
        "*                                                                         *\n",
        "***************************************************************************\n",
        "Usage: ", filename, " model_filename.pcd scene_filename.pcd [Options]\n",
        "Options:\n",
        "     -h:                     Show this help.\n",
        "     -k:                     Show used keypoints.\n",
        "     -c:                     Show used correspondences.\n",
        "     -r:                     Compute the model cloud resolution and multiply\n",
        "                             each radius given by that value.\n",
        "     --algorithm (Hough|GC): Clustering algorithm used (default Hough).\n",
        "     --model_ss val:         Model uniform sampling radius (default 0.01)\n",
        "     --scene_ss val:         Scene uniform sampling radius (default 0.03)\n",
        "     --rf_rad val:           Reference frame radius (default 0.015)\n",
        "     --descr_rad val:        Descriptor radius (default 0.02)\n",
        "     --cg_size val:          Cluster size (default 0.01)\n",
        "     --cg_thresh val:        Clustering threshold (default 5)\n"
    )


def parseCommandLine(argv):
    # Show help
    if '-h' in argv:
        showHelp(argv[0])
        exit(0)

    # Model & scene filenames
    filenames = [i for i in argv if i.split('.')[-1] == 'pcd']
    if len(filenames) != 2:
        print('Filenames missing.')
        showHelp(argv[0])
        exit(-1)

    filenames_params['model_filename_'] = filenames[0]
    filenames_params['scene_filename_'] = filenames[1]

    if '-k' in argv:
        params['show_keypoints_'] = True
    if '-c' in argv:
        params['show_correspondences_'] = True
    if '-r' in argv:
        params['use_cloud_resolution_'] = True

    if '--algorithm' in argv:
        index = sys.argv.index('--algorithm') + 1
        if sys.argv[index] == 'Hough':
            params['use_hough_'] = True
        elif sys.argv[index] == 'GC':
            params['use_hough_'] = False
        else:
            print('Wrong algorithm name.')
            showHelp(argv[0])
            exit(-1)
    # General parameters
    parse_argument(argv, '--model_ss', 'model_ss_')
    parse_argument(argv, '--scene_ss', 'scene_ss_')
    parse_argument(argv, '--rf_rad_', 'rf_rad_')
    parse_argument(argv, '--descr_rad_', 'descr_rad_')
    parse_argument(argv, '--cg_size_', 'cg_size_')
    parse_argument(argv, '--cg_thresh_', 'cg_thresh_')


def computeCloudResolution(cloud):
    res = 0.0
    n_points = 0
    nres = 0
    indices = pcl.vectors.Int()
    sqr_distances = pcl.vectors.Float()
    tree = pcl.search.KdTree.PointXYZRGBA()
    tree.setInputCloud(cloud)

    for i in range(cloud.size()):
        if cloud.at(i).x is np.NAN:
            continue
        # 考虑其第二个邻居，因为第一个是点本身。
        nres = tree.nearestKSearch(cloud.at(i), 2, indices, sqr_distances)
        if nres == 2:
            res += np.sqrt(sqr_distances[1])
            n_points += 1
    if n_points != 0:
        res /= n_points
    return res


if __name__ == '__main__':
    parseCommandLine(sys.argv)
    model = pcl.PointCloud.PointXYZRGBA()
    model_keypoints = pcl.PointCloud.PointXYZRGBA()
    scene = pcl.PointCloud.PointXYZRGBA()
    scene_keypoints = pcl.PointCloud.PointXYZRGBA()
    model_normals = pcl.PointCloud.Normal()
    scene_normals = pcl.PointCloud.Normal()
    model_descriptors = pcl.PointCloud.SHOT352()
    scene_descriptors = pcl.PointCloud.SHOT352()

    # 加载点云文件
    if pcl.io.loadPCDFile(filenames_params['model_filename_'], model) < 0:
        print('Error loading model cloud.')
        showHelp(sys.argv[0])
        exit(-1)
    if pcl.io.loadPCDFile(filenames_params['scene_filename_'], scene) < 0:
        print('Error loading scene  cloud.')
        showHelp(sys.argv[0])
        exit(-1)

    print(model.size(), scene.size())
    # 建立分辨率不变性
    if params['use_cloud_resolution_']:
        resolution = float(computeCloudResolution(model))
        if resolution != 0.0:
            params['model_ss_'] *= params['model_ss_']
            params['scene_ss_'] *= params['scene_ss_']
            params['rf_rad_'] *= params['rf_rad_']
            params['descr_rad_'] *= params['descr_rad_']
            params['cg_size_'] *= params['cg_size_']

        print('Model resolution:       ', resolution)
        print('Model sampling size:    ', params['model_ss_'])
        print('Scene sampling size:    ', params['scene_ss_'])
        print('LRF support radius:     ', params['rf_rad_'])
        print('SHOT descriptor radius: ', params['descr_rad_'])
        print('Clustering bin size:    ', params['cg_size_'])

    # 计算法线
    norm_est = pcl.features.NormalEstimationOMP.PointXYZRGBA_Normal()
    norm_est.setKSearch(10)
    norm_est.setInputCloud(model)
    norm_est.compute(model_normals)

    norm_est.setInputCloud(scene)
    norm_est.compute(scene_normals)

    # 下采样提取关键点
    uniform_sampling = pcl.filters.UniformSampling.PointXYZRGBA()
    uniform_sampling.setInputCloud(model)
    uniform_sampling.setRadiusSearch(params['model_ss_'])
    uniform_sampling.filter(model_keypoints)
    print('Model total points: ', model.size(), '; Selected Keypoints: ', model_keypoints.size())

    uniform_sampling.setInputCloud(scene)
    uniform_sampling.setRadiusSearch(params['scene_ss_'])
    uniform_sampling.filter(scene_keypoints)
    print('Model total points: ', scene.size(), '; Selected Keypoints: ', scene_keypoints.size())

    # 计算关键点描述子
    descr_est = pcl.features.SHOTEstimationOMP.PointXYZRGBA_Normal_SHOT352_ReferenceFrame()
    descr_est.setRadiusSearch(params['rf_rad_'])

    descr_est.setInputCloud(model_keypoints)
    descr_est.setInputNormals(model_normals)
    descr_est.setSearchSurface(model)
    descr_est.compute(model_descriptors)

    descr_est.setInputCloud(scene_keypoints)
    descr_est.setInputNormals(scene_normals)
    descr_est.setSearchSurface(scene)
    descr_est.compute(scene_descriptors)

    # 使用kdtree计算Model-Scene对应
    # model_scene_corrs = pcl.Correspondences() # Correspondences类未完成
    match_search = pcl.search.KdTree.SHOT352()
    match_search.setInputCloud(model_descriptors)

    # 对于每个sence关键点描述符，在模型关键点描述符云中找到最近邻，并将其添加到对应向量中。
    for i in range(scene_descriptors.size()):
        neigh_indices = pcl.vectors.Int()
        neigh_sqr_dists = pcl.vectors.Float()
        if scene_descriptors.at(i).descriptor is np.NAN:
            continue
        found_neighs = match_search.nearestKSearch(scene_descriptors.at(i), 1, neigh_indices, neigh_sqr_dists)
        # 仅当平方描述符距离小于0.25时添加匹配(SHOT描述符距离设计为0和1之间)
        if found_neighs == 1 and neigh_sqr_dists[0] < 0.25:
            corr = pcl.Correspondence(neigh_indices[0], i, neigh_sqr_dists[0])
            # model_scene_corrs.push_back(corr)
   #  print('Correspondences found: ', len(model_scene_corrs))

    viewer = pcl.visualization.CloudViewer('Cluster viewer')
    # viewer.showCloud(model)
    viewer.showCloud(scene)
    while not viewer.wasStopped(10):
        pass


