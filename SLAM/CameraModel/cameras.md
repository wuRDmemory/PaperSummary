# 相机模型的总结

本文主要对常用的相机模型进行总结，主要分为两个部分，针孔模型和鱼眼模型，每个相机模型会有理论介绍与实际的操作，大多数会用到opencv作为工具；

&nbsp;

-----

## 针孔相机模型

TODO



&nbsp;

------

## 鱼眼相机模型

### 参考

1. https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html opencv的fisheye model
2. A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses 一种通用的鱼眼模型
3. https://blog.csdn.net/github_39582118/article/details/115874725 一元多次方程求解根的特征值方法

&nbsp;

### 小孔模型的改进版模型

常用的一些鱼眼模型通常都是在小孔模型的基础上衍生而来，这类模型通常认为畸变的入射角 $\theta_d$ 与真实的入射角 $\theta$ 之间满足一个多项式的关系，如下：
$$
\theta_d=\theta+k_1\theta^{3}+k_2\theta^{5}+k_3\theta^{7}+k_4\theta^{9}+\dots
$$
有了畸变的入射角，根据模型的建模方式不同，可以得到不同的畸变距离：
$$
\begin{aligned}
r&=f\mathrm{tan(\theta)} \quad \text{(i. perspective projection)} \\
r&=f\mathrm{tan(\frac{\theta}{2})}  \quad \text{(ii. stereographic projection)} \\
r&=f\mathrm{\theta}  \quad \quad \quad \text{(iii. equidistance projection)} \\
r&=f\mathrm{sin(\frac{\theta}{2})} \quad \text{(iv. equisolid projection)} \\
r&=f\mathrm{sin(\theta)} \quad \text{ (v. orthogonal projection)} \\
\end{aligned}
$$
图示如下：

<img src='imgs/1.png' width=700/>

左图有两个点是比较重要的，第一个就是 $\theta = \frac{\pi}{2}$ ，图中标记了出来，第二个就是 $r=1$ 。

- $\theta = \frac{\pi}{2}$ 表示入射角可以达到与光轴垂直的角度，可以看到iii, iv, v的模型基本上都可以涵盖这样的情况；
- $r=1$ 表示归一化平面的点与光心的距离为1，此时在x y轴上已经达到了最大了，但是在非轴上的点，它的距离还可以再远一些，这也就是我们通常看到矫正之后的图像是一张四个角被拉伸的图像的原因；

等距模型是最常用的鱼眼模型，也是opencv中使用的鱼眼模型，下面主要介绍该模型，其他模型其实是类似的，只不过通过畸变点与光心距离换算畸变入射角的时候采用不同的公式；

&nbsp;

### 等距模型（Equidistance）

