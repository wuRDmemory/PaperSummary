# 第二章——2D射影几何和变换



## 2D射影空间下的点和线的表示

### 2D射影空间下的线的表示

2D射影空间的线依保持线在2D平面下的参数表示，为：
$$
\mathbf{I}=\begin{bmatrix}
a\\ b\\c
\end{bmatrix} \tag{1}
$$
&nbsp;

### 2D射影空间下的点的表示

2D射影空间由一个二维点的齐次坐标表示：如2D欧式坐标下的[x, y]在射影空间下的表示为：
$$
\mathbf{p}=\begin{bmatrix}
x \\ y\\ 1
\end{bmatrix} \tag{2}
$$
这样表示的优点为：

1. 可以自然的表示无穷远点；
2. 将点和线的维度在2D射影空间下统一，均为3维向量；
3. 方便进行矩阵（向量）运算；

&nbsp;

### 点与线的关系

对于2D射影空间（齐次坐标系）下的点与线，有以下的关系：

- 点 x 在线 l 上当且仅当$\mathbf{x^Tl=0}$;
- 过两点 x1 和 x2 的直线为$\mathbf{l=x1 \times x2}$;
- 两条线段 l1 和 l2 的交点为 $\mathbf{x = l1 \times l2}$;

&nbsp;

### 理想点与无穷远直线

在点的表示中讲到，齐次坐标能够很自然的表示无穷远点（理想点），表示方法如下：
$$
\mathbf{p_{\infty}}=\begin{bmatrix}x\\y\\0\end{bmatrix} \tag{3}
$$
其在**非齐次坐标系**下的值为：$\mathbf{p_{\infty}}=[x/0, y/0]^{T}$，值得注意的是，该点不会映射到任何的欧式坐标上（个人的理解为，如果在欧式坐标表示无穷远点，那么两个坐标不一定都要具有无穷大的坐标值）！在数值意义下仅仅表示该点具有无穷大的坐标值！

有了无穷远点的定义之后，无穷远线也就呼之欲出了：**在2D射影空间里，所有的无穷远点都在无穷远直线上！** 因此如果无穷远点的坐标如公式（3）所示，那么无穷远直线必然与所有的无穷远点的关系均为$\mathbf{I_{\infty}^Tx_{\infty}}=0$，所以无穷远直线为：
$$
\mathbf{I_{\infty}}=\begin{bmatrix}0\\0\\1\end{bmatrix} \tag{4}
$$
&nbsp;

### 射影空间的3D表示

这部分是笔者认为比较重要的部分，因为该部分比较形象，且后面有一些理论配合着该部分能很快的被理解。

在2D射影空间中，所有的点的坐标均为三维坐标，如果抛弃无穷远点的话，所有的点都位于x3=1的平面上，因此可以用三维坐标系中的x3=1的平面来表示所有点的分布平面，如下：

<img src="pictures/1.png">

根据无穷远点的定义，所有的无穷远点均位于线x3=0的x-y平面上，由于点在线上的定义为两个向量的点积为0，在三维坐标系中等价于两个三维向量相互垂直，因此无穷远直线就是z轴，方向向量为[0,0,1]。

&nbsp;

-----

## 2D射影空间下的变换

2D射影空间的变换主要研究的是点与点的变换关系，下面按照书中介绍有：

- 射影变换和透视变换
- 等距变换
- 相似变换
- 仿射变换

### 射影变换（Projective transformation）

射影变换算是射影空间中覆盖范围最广泛的**线性变换（这里不考虑非线性变换）**了，该变换的要求也很简单：射影变换是$\mathbf{IP^2}$到$\mathbf{IP^2}$自身的一种满足下列条件的可逆映射：三点x1，x2，x3共线当且仅当h(x1)，h(x2)，h(x3)也共线，因此射影变换也被称之为保线变换，但是我们更熟悉的名字叫做**单应变换**。

射影变换自身组成了一个映射群：

​	A. 射影变换的逆是射影变换；

​	B. 两个射影变换的复合也是射影变换；

射影变换的一般形式为：
$$
\mathbf{x^{\prime}}=\begin{bmatrix}x1^{\prime}\\x2^{\prime}\\x3^{\prime}\end{bmatrix}=
\begin{bmatrix}h_{11}&h_{12}&h_{13}\\h_{21}&h_{22}&h_{23}\\h_{31}&h_{32}&h_{33}\end{bmatrix}
\begin{bmatrix}x1\\x2\\x3\end{bmatrix}=\mathbf{H_{P}x}  \tag{5}
$$
其中H就是我们熟悉的单应性矩阵了，这里它的自由度为8，是一个齐次矩阵。

结合上面射影空间的3D表示，我们很容易发现：其实单应性矩阵在三维坐标系下与其中x3=1平面上的点进行乘积的话就是对这些点进行了整体的线性变换！换一种说法就是把x3=1这个平面进行了线性变换！如下图：

<img src="pictures/2.png">

透视变换是在射影变换的基础上添加了一些约束得到的一种子变换，MVG书中对于透视变换仅仅是一笔带过的讲了一下，笔者认为主要是该变换还不足够特别到单独拿出来讲，因此一般情况下除了下述的几个特别变换外，基本都使用射影变换。

相较于射影变换，透视变换增加了对于匹配点连线交点的约束，即所有的匹配点的连线交点均交于中心投影的点。这个特点其实给我们提供了两点信息：

- 经过透视变换之后的所有点都必须大于0，否则从原点射出的射线不可能交于变换之后的平面；
- 经过透视变换之后的点与原先点的连线和其他的连线必须是相交的；

因此书中说对于透视投影，整个自由度为6；

> PS: 当把2D射影变换当做是一个线性变换的时候，笔者就比较清晰的明白了为什么说当所有点处于一个平面的时候使用单应性矩阵（射影变换）进行初始化比较好——因为当两个像（图像点）的原像（空间点）都处于三维空间中的同一平面上时，那么其实两对**原像和像之间（3D-2D点对）**的射影变换复合成了**两个像（2D-2D）**的射影变换（单应性矩阵），如下图：
>
> <img src="pictures/4.png">
>
> 如果原像不在同一平面上，那么原像和像的匹配就不能构成射影变换。

&nbsp;

### 等距变换（Equidistant transformation）

等距变换如同名字所述，对于点与点之间的距离是不会修改的，只是对点的位置进行了旋转和平移；

等距变换的公式如下：
$$
\mathbf{x^{\prime}}=\begin{bmatrix}x1^{\prime}\\x2^{\prime}\\1\end{bmatrix}=
\begin{bmatrix}\xi cos(\theta) &-sin(\theta)&t_x\\\xi sin(\theta)&cos(\theta)&t_y\\0&0&1\end{bmatrix}
\begin{bmatrix}x1\\x2\\1\end{bmatrix}=\mathbf{H_{E}x}  \tag{6}
$$
其中$\xi=1$是保向变换，$\xi=-1$是逆向变换（笔者始终没有理解此中的逆向是什么意思）。其中保向变换又被称为是欧式变换，也是我们比较常见的变换：
$$
\mathbf{x^{\prime}}=\begin{bmatrix}x1^{\prime}\\x2^{\prime}\\1\end{bmatrix}=
\begin{bmatrix}R&t\\0&1\end{bmatrix}
\begin{bmatrix}x1\\x2\\1\end{bmatrix}=\mathbf{H_{E}x}  \tag{7}
$$


&nbsp;

### 相似变换（Similarity transformation）

相似变换也和名字叙述的一样，保持了原像和像的相似性（参考相似三角形），那么对于点之间的距离一定是修改了，但是线之间的几何关系（角度、相交性质等）都没有改变；

相似变换的公式如下：
$$
\mathbf{x^{\prime}}=\begin{bmatrix}x1^{\prime}\\x2^{\prime}\\1\end{bmatrix}=
\begin{bmatrix}\mathrm{s} cos(\theta) &-\mathrm{s}  sin(\theta)&t_x\\\mathrm{s}  sin(\theta)&\mathrm{s} cos(\theta)&t_y\\0&0&1\end{bmatrix}
\begin{bmatrix}x1\\x2\\1\end{bmatrix}=\begin{bmatrix}\mathrm{s}R&t\\0&1\end{bmatrix}=\mathbf{H_{S}x}  \tag{8}
$$
这个部分书中引入了一个术语叫做**度量结构**：笔者个人的理解是说当像和原像之间仅仅差一个相似变化的时候，其原本的结构（线之间的夹角、平行度等）都已经是相似的了，这时候很大程度上就可以推测出原像的性质了；同时，该结构也是后面恢复图像时候经常涉及的一个知识点。

&nbsp;

### 仿射变换（Affine transformation）

仿射变换就没有名字一样那么容易记忆了，该变换由一个非奇异变换和平移变换复合：
$$
\mathbf{x^{\prime}}=\begin{bmatrix}x1^{\prime}\\x2^{\prime}\\1\end{bmatrix}=
\begin{bmatrix}a_{11} & a_{12} & t_x\\ a_{21} & a_{22} &t_y\\0&0&1\end{bmatrix}
\begin{bmatrix}x1\\x2\\1\end{bmatrix}=\begin{bmatrix}A&t\\0&1\end{bmatrix}\begin{bmatrix}\mathbf{x}\\1\end{bmatrix} =\mathbf{H_{A}x}  \tag{9}
$$
其中A是一个**非奇异矩阵**，一个有用的结论是A矩阵总是可以分解为旋转和拉伸的复合：
$$
A=UDV^{T}=(UV^{T})(V\Sigma V^{T})=\mathbf{R(\theta)(R(-\phi)\Sigma R(\phi))} \tag{10}
$$
笔者认为该公式能很好的反映出来仿射变换在做的事情：旋转->拉伸->旋转，如下图所示：

<img src="pictures/3.png">

下面探讨一下仿射变换对于原像的改变，也是整本书里面提到很多的不变量：

1. 线段之间的平行关系，这个很重要，因为仿射变换的最后一行为[0,0,1]就是为了不改变无穷远直线，如果原像的无穷远直线被移动了，那么就可以认为原像的平行关系被改变了；
2. 平行线段的长度比，这个也比较容易理解，因为整个仿射变换被分解为了旋转和拉伸，旋转不改变线段的长度，而拉伸这个操作对于平行的向量的作用是一致的；
3. 面积比，这个跟2的分析差不多，都是源于拉伸操作；

&nbsp;

### 小结

书中给出了一个表格对上面的所有的变换进行总结，表格里面也很详尽了，这里不做过多赘述：

<img src="pictures/5.png">

&nbsp;

-----

## 射影变换的分解

这个部分也比较重要，虽然书中的篇幅不多，但是给出的分解值得我们注意，这里直接给出分解的公式：
$$
\mathrm{H}=\mathrm{H}_{\mathrm{s}} \mathrm{H}_{\mathrm{A}} \mathrm{H}_{\mathrm{p}}=\left[\begin{array}{cc}
s \mathrm{R} & \mathbf{t} / v \\
\mathbf{0}^{\top} & 1
\end{array}\right]\left[\begin{array}{cc}
\mathrm{K} & \mathbf{0} \\
\mathbf{0}^{\top} & 1
\end{array}\right]\left[\begin{array}{cc}
1 & 0 \\
\mathbf{v}^{\top} & v
\end{array}\right]=\left[\begin{array}{cc}
\mathrm{A} & \mathbf{t} \\
\mathbf{v}^{\top} & v
\end{array}\right] \tag{11}
$$
可以看到对于一个一般的射影变换，其可分解为三个变换的复合（下面按照变换顺序分析）：

1. 最“简单的”射影变换$\mathbf{H_p}$（2Dof，其中v一般为1），仔细看该变换的话可以看到，抛开一般的$\mathbf{IP^2}$的有限点不说，其改变了无穷远点，进而把无穷远直线变为了有限远直线，也就是改变了平行线的交点；
2. 最“简单的”仿射变换$\mathbf{H_A}$（2Dof），其中K是归一化的上三角矩阵（detK=1，减去了三变量的一个自由度），如果按照上面对于仿射变换的分解，那么该变换就仅仅影响原像的夹角（旋转-拉伸-旋转）以及线段长度（拉伸操作），面积其实并没有放大；
3. 标准的相似变换$\mathbf{H_{s}}$，这个部分就负责整体的旋转，缩放和平移了；

&nbsp;

---

## 射影变换对于线段的变换

上面所有的变换都是对 $\mathbf{IP^2}$ 的点进行的变换，但是这些变换也能同等的对线段进行变换，这里仅仅考虑射影变换，公式如下：
$$
\mathbf{I^{\prime}}=\mathbf{(H^{-1})^{T}}\mathbf{I} \tag{12}
$$
这个推到用点在直线上的公式就很容易得到，这里不赘述；

&nbsp;

----

## 从图像恢复仿射和度量性质

从标题上可以明显看出，这个部分其实讲了两件事，结合射影变换的分解公式：

- 从像的图像去除修改平行关系的射影变换部分$\mathbf{H_{P}}$——仿射矫正；
- 从上一步的图像进一步去除修改角度和长度的仿射变换部分$\mathbf{H_{A}}$——度量矫正；

经过上两步之后，整个射影变换仅仅剩余一个相似变换。

下一件关注的点就是怎么解决了，主要涉及到两个知识点（敲黑板）：

- 无穷远直线的矫正——去除射影变换；
- 虚圆点的矫正——去除仿射变换；

### 无穷远直线的矫正——去除射影变换

> 在书中，这个部分给出了两种解决方案，第二种用距离的方案读者可以自行了解，这里仅仅介绍第一种方案，个人感觉应该也是最常用的方案

这一步的理论也很简单，因为对于原像（或者说是被拍照的对象）而言，其中的两条平行线的交点必然在无穷远直线 $\mathbf{I_{\infty}=[0,0,1]^{T}}$ 上，因此如果像中的两条平行线的交点所确定的直线为 $\mathbf{I_{\infty}^{\prime}}=[l_1, l_2, l_3]^{T}$ 时，那么可以认为该直线完全是因为最简单的射影变换而移动的，公式如下：
$$
\mathbf{I_{\infty}}=\begin{bmatrix}0 \\ 0 \\ 1\end{bmatrix} =
(\left[\begin{array}{cc}
1 & 0 \\
\mathbf{v}^{\top} & v
\end{array}\right]^{-1})^{T}\begin{bmatrix} l_1 \\ l_2 \\ l_3 \end{bmatrix} = 
\begin{bmatrix}
\mathbf{I} & -\frac{\mathbf{V^{T}}}{v} \\ 0 & \frac{1}{v}
\end{bmatrix}\begin{bmatrix} l_1 \\ l_2 \\ l_3 \end{bmatrix}
= (\mathbf{H_P}^{-1})^{T}\mathbf{I_{\infty}^{\prime}} \tag{13}
$$
由此可以推断出：
$$
\mathbf{H_P}=
\begin{bmatrix}1 & 0 & 0 \\ 0 & 1 & 0 \\ l_1 & l_2 & l_3 \end{bmatrix} \tag{14}
$$
需要注意的是：该射影矩阵是把 **像** 映射回 **原像** 的。

&nbsp;

### 虚圆点的矫正——去除仿射变换

#### 虚圆点

首先，什么是虚圆点？用书上的概念说：任何圆都与无穷远直线相交于虚圆点。可以看到这个概念又是一个很数学的概念，和无穷远点一样，仅仅是一个数值上的概念。这里仅仅给出两个虚圆点的定义，对于仅有相似变换的图像来讲，上面的任何圆都与无穷远直线相交于如下的虚圆点：
$$
\mathbf{I}=\begin{bmatrix}1 \\ i \\ 0\end{bmatrix} \quad \mathbf{J}=\begin{bmatrix}1 \\ -i \\ 0\end{bmatrix} \tag{15}
$$
至于为什么说仅有相似变换，读者可以自行看书上的推论，这里不赘述。从“最简单”的仿射矩阵$\mathbf{H_P}$中我们可以看到，仿射变换是会移动虚圆点的，所以一旦我们把虚圆点移动回来了，那么整个仿射变换的影响也就消除了。

所以，如何移动虚圆点？这是一个比较难的地方，我们不可能期望在 像 中找到移动后的虚圆点，之后移动回来。于是就有下面的方法。

#### 虚圆点的对偶二次曲线

这个二次曲线其实是一个3x3的矩阵（参考该章节讲曲线部分），因为这个是为了引出一些结论，这里直接给出公式，设虚圆点的对偶二次曲线为 $\mathbf{C_{\infty}}$，其公式为：
$$
\mathbf{C_{\infty}}=\mathbf{IJ^{T}+JI^{T}}=
\begin{bmatrix}1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 0\end{bmatrix}
\tag{16}
$$
将点的射影变换公式$\mathbf{x^{\prime}}=\mathbf{Hx}$ 应用到该对偶二次曲线可以发现：
$$
\mathbf{C_{\infty}^{\prime}}=\mathbf{HIJ^{T}H^{T}+HJI^{T}H^{T}}=\mathbf{HC_{\infty}H^{T}} \tag{17}
$$
还有值得注意的一点是，对于相似变换而言，虚圆点对偶二次曲线是不会被改变的，这个结论之所以重要是因为当逐层的去除射影变换和仿射变换之后，剩下相似变换的 **像** 的对偶二次曲线就已经和 **原像** 的二次曲线一样了，均为公式（16）的形式；



#### 射影平面上两条直线的夹角公式

笔者这里也没有理解为什么公式是这样的，所以也直接给出射影变换下**不变的**夹角公式为：
$$
cos(\theta)=\frac{\mathbf{L^{T}C_{\infty}m}}{\sqrt{(\mathbf{L^{T}C_{\infty}L})(\mathbf{m^{T}C_{\infty}m})}} \tag{18}
$$
其中十分有必要理清楚变量所属的系的问题：

1. $\mathbf{L}， \mathbf{m}$ 均是在 **像** 上的两条直线；
2. $\mathbf{C_{\infty}}$ 也是 **像** 上的虚圆点的对偶二次曲线；
3. $\theta$ 是 **原像（可以认为就是欧式空间）** 上的夹角；

该公式之所以具有**不变性**，主要是因为把射影变换对于线和虚圆点对偶曲线的公式带入就能得到不变的结论。

#### 恢复度量性质

由公式（18）很容易得出，如果在 **像** 上选择两条在 **原像** 上垂直的线，那么就可以通过 $\mathbf{L^T C_{\infty} m}=0$ 来确定在 **像** 上的虚圆点对偶二次曲线，再由公式（17）反算出来对应的仿射变换$\mathbf{H_A}$（前提是已经进行了仿射矫正）。

例如对于已经进行了仿射矫正的图像来说，选取其中在欧式坐标系下垂直的两条线段 $\mathbf{L}=[l_1, l_2, l_3]^{T}$ 和 $\mathbf{m}=[m_1, m_2, m_3]^{T}$，所以由公式（18）可以得到：
$$
[l_1, l_2, l_3]\mathbf{C_{\infty}^{\prime}}\begin{bmatrix} m_1 \\ m_2 \\ m_3 \end{bmatrix} = \mathbf{0} \tag{19}
$$
其中 $\mathbf{C_{\infty}^{\prime}}$ 是在 **像** 上的虚圆点对偶二次曲线，如果 **像** 已经进行了仿射矫正，那么其与 **原像（也就是欧式空间）** 之间仅仅相差一个仿射变换$\mathbf{H_A}$，所以 **像** 上的虚圆点对偶二次曲线可以由公式（17）推导得到：
$$
\mathbf{C_{\infty}^{\prime}}=\mathbf{H_{A}C_{\infty}H_A^{T}}=
\begin{bmatrix} K & 0 \\ 0 & 1 \end{bmatrix}\begin{bmatrix} I & 0 \\ 0 & 0 \end{bmatrix}\begin{bmatrix} K^{T} & 0 \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} KK^{T} & 0 \\ 0 & 0 \end{bmatrix} \tag{20}
$$
如上面的分析，因为 K 是一个归一化的上三角矩阵，所以 $KK^{T}$ 是一个对称矩阵，仅有三个自由度，于是可以简化为：
$$
KK^{T}=\begin{bmatrix} s_1 & s_2 \\ s_2 & s_3 \end{bmatrix} \tag{21}
$$


将公式（20）（21）代入公式（19）可以得到：
$$
\begin{aligned}
\begin{bmatrix}l_1 & l_2\end{bmatrix}\begin{bmatrix}s_1 & s_2 \\ s_2 & s_3\end{bmatrix} \begin{bmatrix}m_1 \\ m_2\end{bmatrix} &= 0 \\
l_1m_1s_1+(l_1m_2+l_2m_1)s_2+l_2m_2s_3&=0 \\ 
\begin{bmatrix}l_1m_1 & l_1m_2+l_2m_1 & l_2m_2\end{bmatrix}\begin{bmatrix}s_1 \\ s_2 \\ s_3\end{bmatrix}&=0
\end{aligned} \tag{22}
$$
公式（22）告诉我们：一对在欧式空间中垂直的直线能够提供一个零空间的约束，所以两对垂直线就可以提供两个零空间的约束，如下：
$$
\begin{bmatrix}l_1^{1}m^{1}_1 & l^{1}_1m^{1}_2+l^{1}_2m^{1}_1 & l^{1}_2m^{1}_2 \\ l_1^{2}m^{2}_1 & l^{2}_1m^{2}_2+l^{2}_2m^{2}_1 & l^{2}_2m^{2}_2\end{bmatrix}\begin{bmatrix}s_1 \\ s_2 \\ s_3\end{bmatrix}=\mathbf{Ax}=0 \tag{23}
$$
上面的方程很好求解，一旦求解了$s_1, s_2, s_3$，就可以直接构建 $KK^{T}$ 矩阵，之后通过cholesky分解得到 K 矩阵，就得到了仿射矩阵$\mathbf{H_A}$。

注意该仿射矩阵是把 **原像** 映射为 **像** 的。

&nbsp;

----

## 仿真实验

这里用书中的地砖例子做实验，**像** 如下：

<img src="pictures/example/2.png">

### 仿射矫正

在进行仿射矫正的步骤，基本上可以随便取两条平行的线段（但还是建议按书中的做法，取相互垂直的线段），代码如下：

```c++
void affineCalibr(const vector<cv::Point>& horizon_line_points) {
    assert(horizon_line_points.size() == 8);

    vector<Eigen::Vector3d> inf_point_in_inf_line;
    for (int i = 0; i < 2; ++i) {
        Eigen::Vector3d p0(horizon_line_points[4*i+0].x, horizon_line_points[4*i+0].y, 1.);
        Eigen::Vector3d p1(horizon_line_points[4*i+1].x, horizon_line_points[4*i+1].y, 1.);
        Eigen::Vector3d p2(horizon_line_points[4*i+2].x, horizon_line_points[4*i+2].y, 1.);
        Eigen::Vector3d p3(horizon_line_points[4*i+3].x, horizon_line_points[4*i+3].y, 1.);

        Eigen::Vector3d line1_dir = (p0.cross(p1)).normalized();
        Eigen::Vector3d line2_dir = (p2.cross(p3)).normalized();

        inf_point_in_inf_line.push_back(line1_dir.cross(line2_dir));
    }

    assert(inf_point_in_inf_line.size() == 2);

    Eigen::Vector3d inf_line_dir = (inf_point_in_inf_line[0].cross(inf_point_in_inf_line[1])).normalized();
    //! must convert l3's sign is positive
    //! or the orientation of your transform is wrong.
    if (inf_line_dir.z() < 0) {
        inf_line_dir *= -1;
    }
    cv::Mat H = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, inf_line_dir[0], inf_line_dir[1], inf_line_dir[2]);
    cout << H << endl;

    cv::Mat trans_image;
    cv::warpPerspective(image, trans_image, H, image.size());

    cv::imshow("[calibr]", trans_image);
    cv::waitKey();
}
```

在实现的过程中，这里面一个比较大的坑是当求出的 **像** 上无穷远直线的方向与 z 轴正方向（[0,0,1]）相反的时候，一定需要把方向转回来！否则相当于你认为 **像** 的成像平面是在 z 的负方向！这时候变换矩阵会把本在 z 轴正方向的 **像** 映射到 z 轴负方向，导致图像特别小（或者说缩成了一点）。

结果如下：

<img src="pictures/example/2_calibr.png">

### 度量矫正

在进行度量矫正的时候，实验的经验是不能随意取垂直线段了，最好是同时**取两组夹角不同的在 原像 上垂直向量**，注意这里一定是向量哈，线段没有方向，但是向量是有方向的，比如如果我们围绕着一个在 **原像** 上是矩形的砖块**顺时针**取垂直向量，那么其实两个的夹角是一样的，如下图中黑线以及对应的红线向量，正确的方法应该是修改其中一向量的方向就把其中一个约束的夹角变为了钝角，如图中路线所示：

<img src="pictures/6.png">

代码如下所示：

```c++
void metricRecovery(const vector<cv::Point>& orthogonal_line_points) {
    assert(orthogonal_line_points.size() == 8);

    vector<pair<Eigen::Vector3d, Eigen::Vector3d>> orthogonal_lines;
    for (int i = 0; i < 2; ++i) {
        Eigen::Vector3d p0(orthogonal_line_points[4*i+0].x, orthogonal_line_points[4*i+0].y, 1.);
        Eigen::Vector3d p1(orthogonal_line_points[4*i+1].x, orthogonal_line_points[4*i+1].y, 1.);
        Eigen::Vector3d p2(orthogonal_line_points[4*i+2].x, orthogonal_line_points[4*i+2].y, 1.);
        Eigen::Vector3d p3(orthogonal_line_points[4*i+3].x, orthogonal_line_points[4*i+3].y, 1.);

        Eigen::Vector3d line1_dir = (p0.cross(p1)).normalized();
        Eigen::Vector3d line2_dir = (p2.cross(p3)).normalized();

        orthogonal_lines.emplace_back(line1_dir, line2_dir);
    }

    assert(orthogonal_lines.size() == 2);

    Eigen::Matrix<double, 2, 3> KKT;
    for (size_t i = 0; i < orthogonal_lines.size(); ++i) {
        const auto&  lm = orthogonal_lines[i];
        const double l1 = lm.first.x(), l2 = lm.first.y();
        const double m1 = lm.second.x(), m2 = lm.second.y();
        KKT.row(i) = Eigen::Vector3d(l1*m1, (l1*m2+l2*m1), l2*m2);
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 2, 3>> svd(KKT, Eigen::ComputeFullV);
    Eigen::Vector3d X = svd.matrixV().col(2);
    Eigen::Matrix2d H_A;
    H_A << X(0), X(1), X(1), X(2);

    //! chelosky decomposition and s.t. det(K)=1
    Eigen::Matrix2d K = H_A.llt().matrixL();
    K /= sqrt(K.determinant());

    cv::Mat H = (cv::Mat_<double>(3, 3) << K(0,0), K(1,0), 0, K(0,1), K(1,1), 0, 0, 0, 1);
    cv::Mat trans_image;

    //! this H maps original image to image, and we have image.
    //! so we need use inverse of it.
    cv::warpPerspective(image, trans_image, H.inv(), image.size());

    cv::imshow("[recovery]", trans_image);
    cv::waitKey();
}
```

结果如下：

<img src="pictures/example/2_recovery.png">