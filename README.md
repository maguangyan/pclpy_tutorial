# pclpy中文教程

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

* Features
* Filtering
* I/O
* Keypoints
* KdTree
* Octree
* Range Images
* Recognition
* Registration
* Sample Consensus
* Segmentation
* Surface
* Visualization
* Applications
* GPU

## 版权信息

除非额外说明，本网站的所有公开文档均遵循 [署名-非商业性使用-相同方式共享 3.0 中国大陆 (CC BY-NC-SA 3.0 CN)](https://creativecommons.org/licenses/by-nc-sa/3.0/cn/) 许可协议。任何人都可以自由地分享、修改本作品，但必须遵循如下条件：

- 署名：必须提到原作者，提供指向此许可协议的链接，表明是否有做修改
- 非商业性使用：不能对本作品进行任何形式的商业性使用
- 相同方式共享：若对本作品进行了修改，必须以相同的许可协议共享
