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
由网络的整体结构图可以看到，整个子网络共预测了3个feature: Heatmaps, Embeddings和Offsets，下面慢慢来分析一下三种feature的作用。

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

## Offsets
经过上一层的Heatmaps，我们就**大概**知道了角点的位置，但是由于网络经过了一个沙漏网络，会进行下采样，每次下采样时的倍率为2，因此假设缩放了n次，此时角点的位置为$[x//2^n, y//2^n]$，但是在该层真实的角点位置为$[x/2^n, y/2^n]$，此时会有一个误差，作者认为这个误差对于较小的物体的IOU会有很大的影响，因此这个误差是必须要被考虑在内的，所以在网络最终有一个Offsets层，该层专门预测在沙漏网络中间层的误差，label为:
$$
\boldsymbol{o}_{k}=\left(\frac{x_{k}}{n}-\left\lfloor\frac{x_{k}}{n}\right\rfloor, \frac{y_{k}}{n}-\left\lfloor\frac{y_{k}}{n}\right\rfloor\right)
$$

预测出来的值与label的值做L1Loss，如下：
$$
L_{o f f}=\frac{1}{N} \sum_{k=1}^{N} \operatorname{SmoothL} 1 \operatorname{Loss}\left(\boldsymbol{o}_{k}, \hat{\boldsymbol{o}}_{k}\right)
$$

**特别注意的是**，作者分别对top-left和bottom-right角点预测了一组Offsets特征图，不同的物体共享同样的offset，这里大胆的猜测一下，对于最终的输出图上的(i, j)点，其对应两个offset特征图，如果(i,j)点上有物体k，则物体k的offset label与该预测计算error。

## Embeddings
根据Heatmaps的信息，现在仅仅知道(i,j)点是物体k的一个角点，但是另一个角点如何寻找？作者在网络的最后预测了一个Embeddings特征图，该特征图的每个点都是一个Embedding向量，如果两个角点属于一个物体，则其Embedding向量的距离应该是最小的。**值得说明的是**，作者对于Embedding的预测也是一个，即所有的物体的角点共享一个Embedding向量（这样的设定个人感觉是合理的，因为特征图最后的分辨率和原始图一样大，不太可能出现两个物体的角点出现在同一个像素中的情况）。

在训练的过程中，对Embedding的训练有两个部分：
$$
L_{\text {pull}}=\frac{1}{N} \sum_{k=1}^{N}\left[\left(e_{t_{k}}-e_{k}\right)^{2}+\left(e_{b_{k}}-e_{k}\right)^{2}\right] \\
L_{\text {push}}=\frac{1}{N(N-1)} \sum_{k=1}^{N} \sum_{j=1 \atop j \neq k}^{N} \max \left(0, \Delta-\left|e_{k}-e_{j}\right|\right)
$$

第一个是pull loss，该loss把成对角点的embedding向量距离缩小，其中$e_k=\frac{(e_{t_{k}}+e_{b_{k}})}{2}$；第二个是push loss，该loss把不成对的角点的embedding向量距离扩大，其中$\Delta=1$。

----
## Corner pooling
corner pooling是作者十分推崇的一种pooling方式，事实也证明这样的方式确实对于Corner Net十分有效。

其原理也比较简单，因为Corner一般分布在真实物体的外部，但是很显然，如果这个点是bbox的一个端点的话，那么**该点的横向或者纵向一定有物体的边界点，这些边界点的特征值一定较高**，那么对于一个点(i, j)，如果综合它的横向和纵向的值，如果该点的横向和纵向都经过物体的边界，那么该点的值一定特别高，所以这个点就是角点。作者给出的说明如下图：

<img src="imgs/2019-09-14 15-59-39.png">

作者举出的是top-left角点的corner pooling操作。数学操作为：

$$
\begin{aligned} t_{i j} &=\left\{\begin{array}{cl}{\max \left(f_{t_{i j}}, t_{(i+1) j}\right)} & {\text { if } i<H} \\ {f_{t_{H j}}} & {\text { otherwise }}\end{array}\right.\\ l_{i j} &=\left\{\begin{array}{cl}{\max \left(f_{l_{i j}}, l_{i(j+1)}\right)} & {\text { if } j<W} \\ {f_{l i W}} & {\text { otherwise }}\end{array}\right.\end{aligned}
$$

作者最后对这部分的网络结构进行了说明，整个预测子网络是对residual block的修改，在拿到主干网络的最后一层输出之后，feature流向两个路线，一个是1×1+BN结构，一个是corner pooling结构，经过两个路线的feature会叠加在一起形成新的feature；随后再经过几个常规的卷积层和激活函数得到三个feature map。子网络的结构图如下：

<img src="imgs/2019-09-14 16-31-33.png">

作者最后对比了一下使用corner pooling和不使用corner pooling的区别，如下：

<img src="imgs/2019-09-14 16-19-16.png">

