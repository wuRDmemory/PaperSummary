# CornerNet

## 总体叙述
CornerNet属于anchor-free方法的一种，个人觉得特点如下：
 - 整个过程中舍弃了anchor；
 - 没有使用多尺度的方法；
 - 提出了corner pooling操作，更有利于对corner的提取；
 - 引入embedding向量来决定哪些corner是属于同一个物体；

## 整体网络结构
CornerNet整体结构如下图：
<img src="imgs/2019-09-11 17-54-54.png">

从图上可以看到，CornerNet有如下特点：
 - 使用一个主干网络；
 - 对主干网络的输出做了两个分支，分别得到corner的位置和对应的embedding向量；

下面对这两个方面进行介绍。

## Detect Corner
由网络的整体结构图可以看到，整个子网络共预测了3个feature: Heatmaps, Embeddings和Offsets，下面慢慢来分析一下为什么作者生成这三种feature.

### Heatmaps
Heatmaps的大小为H×W×C，其中C为网络要预测的种类数，再该特征图的每一个点(i,j)上，都有一个长度为C的vector，**第k个值表示该点是k物体角点的概率**。那么显然，在训练的时候，如果物体k的角点落在了点(i,j)上，那么该点的正例即为k。

反过来，其他的非(i,j)点在k物体的预测上就都要设置为负例吗？我个人觉得这种暴力的方法应该是可行的，但是作者并没有这么做，因为很容易想到，如果相邻坐标$(i+\delta i, j+\delta j)$作为正例其实也能保证较大的IOU，因此作者这里设定了一个半径范围，在该范围内的，随着距离的大小更改其作为负例的权重，作者附上的一幅图如下：
<img src="imgs/2019-09-11 18-45-57.png">

红框为真值，在真值的附近画一个范围(橘色圈)，在该圈范围内的点(i,j)的负例权重用一个2D的非归一化高斯函数表示为：
$$
weight = (1-e^{-\frac{x^{2}+y^{2}}{2 \sigma^{2}}})
$$
其中$x, y$为点(i,j)距离真值的坐标差，$\sigma$为半径的0.333倍，也就是在半径的0.333倍范围内的，其负例权重还是比较小的(可以作为正例)，而在这之外，负例的权重就很大了。

而橘色圈的半径是根据角点的真值确定的，作者给出的建议是两个角点围成的box与真实box的IOU为0.3时(编程角度来说，该半径应该为0.075倍的长宽中的最小值就可以了？)。

**特别注意的是**，使用CornerNet这种策略的时候，无需像有anchor的网络一样，对预测的中心和长宽进行回归，只需要对Heatmaps的每个点按照分类的方式进行train就可以了，作者这里使用的是Focal Loss的变体：
$$
L_{d e t}=\frac{-1}{N} \sum_{c=1}^{C} \sum_{i=1}^{H} \sum_{j=1}^{W}\left\{\begin{array}{c}{\left(1-p_{c i j}\right)^{\alpha} \log \left(p_{c i j}\right) \text { if  }y_{cij}=1} \\ {\left(1-y_{c i j}\right)^{\beta}\left(p_{c i j}\right)^{\alpha} \log \left(1-p_{c i j}\right) \text { otherwise }}\end{array}\right.
$$

可以看到，对于真值对应的点(i,j)，按照Focal loss的方式进行，对于非(i,j)点，使用高斯函数得到权重$(1-y_{cij})$之后，按照负例进行学习。








