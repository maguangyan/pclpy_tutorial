# pclpy教程

版本v0.1

## 动机

pclpy作者没有给出详细的教程和文档，对于没有c++下PCL开发经验的同学们来说，直接上手python版的pclpy可能会有一定难度。鉴于此，本人在学习PCL的基础上，计划参考PCL官方教程撰写pclpy的python教程。

> 本人技术尚浅，如有错误，望通知更正

本仓库与本人[博客文章](https://blog.csdn.net/weixin_44456692)同步更新

## 什么是pclpy

pclpy是点云库(PCL)的Python绑定。使用CppHeaderParser和pybind11从头文件生成。
这个库正在积极开发中，api可能会发生变化。所包含的模块确实可以工作，但测试还不完整。目前只支持Windows和python 3.6 x64。

许多其他python库尝试绑定PCL。最流行的是python-pcl，它使用Cython。虽然Cython非常强大，但绑定c++模板并不是它的强项(PCL大量使用模板)。python-pcl有大量的代码重复，维护和添加特性都非常难，而且对PCL的类和点类型绑定不完整。使用pybind11绑定，我们可以直接使用c++，模板、boost::smart_ptr和缓冲区协议都比较容易实现。

GitHub：https://github.com/davidcaron/pclpy

pypi: https://pypi.org/project/pclpy/

pybind11：https://pybind11.readthedocs.io/en/stable/

https://blog.csdn.net/weixin_44456692/article/details/114854873

## 安装

详情请阅读：[点云处理工具——pclpy安装](https://blog.csdn.net/weixin_44456692/article/details/114854873?spm=1001.2014.3001.5501)

## 大纲

* [I/O](./tutorial/01_IO)
* [KdTree](./tutorial/02_KdTree)
* [Octree](./tutorial/03_Octree)
* [Visualization](./tutorial/04_Visualization)
* [Filtering](./tutorial/05_Filtering)
* [Range Images（跳过）](./tutorial/06_RangeImages)
* [Keypoints（跳过）](./tutorial/07_Keypoints)
* [Sample Consensus](./tutorial/08_SampleConsensus)
* [Features](./tutorial/09_Features)
* [Registration（跳过）](./tutorial/10_Registration)
* [Segmentation](./tutorial/11_Segmentation)
* [Recognition（绑定不完全）](./tutorial/12_Recognition)
* [Surface](./tutorial/13_Surface)
* [Applications](./tutorial/14_Applications)
* GPU（跳过）

## 后记

---

2021.10.10日  记：

历时一个月的时间，今天终于完成了这个仓库的第一个版本，因为时间仓促，这个教程还不完美，若有错误，恳请指正。

想起来，做这个pclpy教程也是心血来潮，因为九月十月开始了秋招，每天都在忙于投简历、面试，在这种情况下整个人都有些躁动，静不下心搞论文。于是为了转移注意力，让找工作的这两个月有点事做，这时候正好看到之前发的一个笔记[三维重建工具——pclpy使用教程](https://blog.csdn.net/weixin_44456692/article/details/115163270) 阅读和点赞量都比较高，而且有很多小伙伴在评论区问问题，我意识到了有很多学python的小伙伴可能对点云处理很感兴趣。于是，心血来潮，开始开干！就有了pclpy教程这个仓库。

简单总结一下在写这个教程过程中的一个体会。pclpy是点云库(PCL)的Python绑定，其使用pybind11绑定，我们可以直接使用c++，模板、boost::smart_ptr和缓冲区协议都比较容易实现。从使用角度来说呢，pclpy和PCL的写法很多地方几乎一致，这使得我们如果写PCL的经验，就会很容易上手pclpy。但是pclpy的缺点也很明显！！！毕竟PCL是一个庞大的算法库，使用pybind11绑定也是一个庞大的工作量，[pclpy](https://github.com/davidcaron/pclpy) 这个库正在积极开发中（作者原话），但是我看到最近作者的更新频率不是很高（最近一次是在2021.03.09），pclpy仓库中显示目前开发进度如下

开发中的模块

- 2d
- common
- geometry
- features
- filters
- io (meshes are not implemented)
- kdtree
- keypoints
- octree
- recognition
- sample_consensus
- search
- segmentation
- stereo
- surface
- tracking

跳过的模块：

- ml
- people
- outofcore
- registration
- visualization
- every module not in the PCL Windows release (gpu, cuda, etc.)

但实际使用过程中，情况没有那么乐观，有很多类还未实现。

对已测试的功能中碰到的问题总结如下：

* [I/O](./tutorial/01_IO) 

点云拼接 pcl::concatenateFields未绑定

* [KdTree](./tutorial/02_KdTree)
* [Octree](./tutorial/03_Octree) 

点云压缩pcl::compression::octree_pointcloud_compression类未绑定 

* [Visualization](./tutorial/04_Visualization) 

runOnVisualizationThreadOnce()未绑定

可视化椭圆viewer.addSphere()报错：no override found for "VTKPolyDataMapper"

可视化法线viewer.addPointCloudNormals(）报错：In..\Rendering\Core\vtkActor.cxx, line 43. Error: no override found for vtkactor

可视化mesh报错：viewer.addPolygon() 这个作者提到过，还未绑定好

* [Filtering](./tutorial/05_Filtering)

条件滤波ConditionalRemoval类未绑定。

* [Range Images（跳过）](./tutorial/06_RangeImages) 

RangeImage类未绑定

* [Keypoints（跳过）](./tutorial/07_Keypoints)

keypoints模块未绑定

* [Sample Consensus](./tutorial/08_SampleConsensus)
* [Features](./tutorial/09_Features)

pcl::MomentOfInertiaEstimation类存在bug，主要是getEigenValues()，getEigenVectors()，getMassCenter()这三个函数，使用numpy数组直接作为参数的函数似乎都有这个问题。

旋转投影统计特征pcl::ROPSEstimation类未绑定成功

GASD类、GRSD类未完成绑定。

* [Registration（跳过）](./tutorial/10_Registration) 

Registration模块未绑定

* [Segmentation](./tutorial/11_Segmentation)

基于颜色的区域生长分割pcl::RegionGrowingRGB类未绑定

条件欧几里得聚类pcl::ConditionalEuclideanClustering类未绑定

基于法线差异的分割pcl::DifferenceOfNormalsEstimation类未绑定

将点云聚类为 Supervoxels的pcl::SupervoxelClustering类存在bug，getSupervoxelAdjacency()函数报错，应该也是未完全绑定。

使用 ModelOutlierRemoval 过滤点云中的pcl::model_outlier_removal类未绑定。

* [Recognition（绑定不完全）](./tutorial/12_Recognition)

未绑定的类：

> pcl::Correspondences
>
> pcl::Hough3DGrouping
>
> pcl::ism::ImplicitShapeModel
>
> pcl::Correspondences
>
> pcl::Hough3DGrouping
>
> ...

* [Surface](./tutorial/13_Surface)

B样条拟合算法pcl::on_nurbs未绑定。

* [Applications](./tutorial/14_Applications)
* GPU（跳过）

总的来说，由于PCL绑定工作量比较大，pclpy目前只绑定了PCL部分功能，玩一玩儿可以，实际项目中作用不是很大，好好学PCL吧。

