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
* Recognition
* Surface
* Applications
* GPU

