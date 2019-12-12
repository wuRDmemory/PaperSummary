# SVO实现细节整理



## feature align中的warp matrix的计算

**提出原因**：在feature align过程中，因为当前帧距离reference frame的距离较远，但是还要使用光度误差来矫正位置，所以作者使用一个warp matrix来对两帧之间的像素块进行仿射变换，期望使得光度误差更接近真实的误差。

**方法步骤**：

- 将同一个3D点投影到reference frame上得到refp，之后对refpx的x和y轴加上halfpatch+1的长度得到refpx和refpy，至此得到三个点: refp, refpx, refpy，这三个点组成的两个向量刚好可以组成基向量；
- 将上一步中的三个点依次投到世界坐标系并投到current frame上得到curp, curpx, curpy，而curpx-curp和curpy-curp就是warp matrix的两个列向量，current frame上的点要和warp matrix相乘得到仿射变换之后的点的坐标；



## align层的选择

**提出原因**：在feature align过程中，得到warp matrix后，需要看一下该矩阵对当前帧的伸缩程度(其实就是矩阵的行列式)，如果伸缩过大，会导致计算光度误差的点特别分散，这时候就需要找到一个仿射程度适中的层进行仿射变换；

**方法步骤**：

- 对于得到warp matrix A，计算行列式S=det(A)，该值表示在金字塔的最底层间的仿射变换程度；
- 如果该值较大，则把level提升一层，相应的，S=S*0.25，循环往复直到level到达最大或者S小于阈值；



## SVO中landmark的变化历程
SVO中的landmark有四种状态，CANDIDATE，UNKNOW，GOOD，DELETE，变化历程如下：
1. CANDIDATE在reproject过程中，投影到当前帧上，如果可以投影上，则暂时把3D点挂载在当前帧上，参与后续帧位姿的ImageAlign
2. 如果当前帧变为关键帧了，则这些挂载的点就会变为UNKNOW，这是CANDIDATE翻身的唯一机会；
3. UNKOWN的点通过reproject过程投影到当前帧上，如果投影的光度误差满足条件，则计数器加1，到达一定程度了，则变为GOOD点；如果比较差，则标记为DELETE，之后一起删除掉；
4. DELETE点则在之后的过程中删掉就可以；

