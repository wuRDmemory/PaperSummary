# 第七章——摄像机矩阵P的计算

---

## 写在前面

本章节主要介绍使用世界点与图像点对应关系求解摄像机矩阵P，之后还会继续进行内外参的分解；其次本章节也讲了图像径向失真的矫正。

&nbsp;

-----

## 基本方程（代数误差）

#### 构建方程

对于给定了3D->2D对应的集合{X}->{x}，于是可以构建映射关系为：
$$
\mathrm{x} = \begin{bmatrix}x \\ y \\ w \end{bmatrix} = \mathrm{PX} =  \begin{bmatrix} \mathrm{P^{1}X} \\ \mathrm{P^{2}X} \\ \mathrm{P^{3}X} \end{bmatrix} \tag{1}
$$
所以$\mathrm{x}\times \mathrm{PX} = 0$，推得：
$$
\left[\begin{array}{ccc}
\mathbf{0}^{\boldsymbol{\top}} & -w_{i} \mathbf{X}_{i}^{\top} & y_{i} \mathbf{X}_{i}^{\top} \\
w_{i} \mathbf{X}_{i}^{\top} & \mathbf{0}^{\mathrm{T}} & -x_{i} \mathbf{X}_{i}^{\top} \\
-y_{i} \mathbf{X}_{i}^{\top} & x_{i} \mathbf{X}_{i}^{\top} & \mathbf{0}^{\top}
\end{array}\right]\left[\begin{array}{c}
\mathbf{P}^{1} \\
\mathbf{P}^{2} \\
\mathbf{P}^{3}
\end{array}\right]=\mathbf{0} \tag{2}
$$
可以看到和第四章求解射影矩阵的DLT方法一致，于是其中第四章很多方法都是可以应用过来，比如数据处理中的归一化技术等等。

因为整个摄像机矩阵的未知量是12，同时因为没有明确的尺度信息（主要是因为图像坐标在齐次坐标系下），所以真正的自由度为11维，公式（2）中1对点对可以提供2个约束，因此至少需要6对对应点对才能求出来摄像机矩阵（当然，如果对应的点对大于6，自然可以求解超定方程得到超定解）。

&nbsp;

#### 添加约束

与第四章的求解一样，我们在求解形如 $\mathrm{Ax=0}$ 这样的方程时，也要加入一定的约束条件适配真正的自由度11：

1. 添加约束条件 $\mathrm{|p|=1}$，其中 $\mathrm{p}$ 表示摄像机矩阵组成的列向量；
2. 添加约束条件 $\mathrm{|\hat{p}^{3}|=1}$，其中 $\mathrm{\hat{p}^{3}}$ 表示摄像机矩阵的 P[3, 0:3] 组成的向量，该部分之所以为1是因为该部分本质上是 KR 的乘积，其向量模为1； 

虽然笔者个人觉得约束2是更加合理的，但是实际上，使用约束1不仅会比较方便，而且求解出来P之后也可以用RQ分解得到可靠的内参矩阵和旋转矩阵。

&nbsp;

#### 数据处理



