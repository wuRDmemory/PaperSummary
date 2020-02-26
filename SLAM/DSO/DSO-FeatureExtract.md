# DSO(1)——FeatureExtract



## DSO简介

DSO是Direct Sparse Odometry的缩写，从名字可以看到，系统使用直接法进行位姿的求解，使用部分而非全部的像素点参与到优化过程中。比较厉害的是，DSO在优化变量中加入了光度校准，其模型考虑了曝光时间，镜头渐晕现象以及光度的非线性响应函数。所以整个系统的代码说起来是比较麻烦的，这里分为多个部分来一一介绍。



## 特征点提取的重要性

在DSO论文的简介中有这样一句话：

> This is achieved in real time by omitting the smoothness prior used in other direct methods and instead sampling pixels evenly throughout the images

大意是说为了达到实时（Excuse me?）而忽略了其他直接法中的平滑先验（这个还真没有接触过），代替的是通过在整幅图上进行均匀采样。可以见得，特征点的提取方法在DSO这个系统中的重要性是不言而喻的。

同时，论文的3.2节也较为详细的说明了点是如何从提取到成熟的过程。针对这个部分，该文就结合着论文和代码一起描述一下作者是如何提取关键点以及背后的优势。



## 特征点提取方法

该总结的主要针对的是工程中Coarseinitializer类的setFirst方法。

### 特点

我个人认为DSO的特征点提取方法具有如下特点：

- 结合图像金字塔进行特征提取；
- 只提取梯度模长超过阈值的点作为特征点；
- 梯度模长的阈值使用自适应阈值；



### 过程





