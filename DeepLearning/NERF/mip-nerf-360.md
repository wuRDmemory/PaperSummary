<!--
 * @Author: Chenhao Wu wuch2039@163.com
 * @Date: 2023-05-18 07:39:26
 * @LastEditors: Chenhao Wu wuch2039@163.com
 * @LastEditTime: 2023-05-19 08:07:03
 * @FilePath: /paper_summary/DeepLearning/NERF/mip-nerf-360.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# Mip-NeRF 360: Unbounded Anti-Aliased Neural Radiance Fields

---

## 写在前面

看标题就能知道：本文是对于Mip-NeRF在无边界场景的应用，同年的网络有Block-NeRF（看引用的话是Mip-NeRF 360在前），不过相比于Block-NeRF，Mip-NeRF 360首先没有那么大，其次主要的探索点也不尽相同。具体而言：
1. Block-NeRF场景更大，Mip-NeRF 360的场景较小；
2. Block-NeRF更多的是使用一些手法将多个Mip-NeRF拼凑在一起，Mip-NeRF360则是进一步的对Mip-NeRF的采样和效果进行改进；

&nbsp;

---

## 详细过程

### 无边界NeRF方法的问题
作者在开头处写到：对于无边界NeRF而言，主要的问题有:
1. 参数较多：因为需要在ray上进行采样，无边界场景就会要求有更多的采样点；
2. 效率：这个自不用说，更大的网络效率自然下降；
3. 歧义性（Ambiguity）：因为远处的场景仅有很少的ray可以采样到，这样的现象加剧了2D图像重建3D内容的固有歧义；

作者在后续的2，3，4章节中对该问题进行了深入的探讨和解决；Mip-NeRF的相关知识已经在前序文章中进行了介绍，这里不多赘述，后面有一张图对该过程进行了比较详细的展示；

> PS：个人认为这里的无边界并不是指大场景，而是说不想原生NeRF一样是对于一个物体进行的重建，这里的无边界更多的是说背景无边界

&nbsp;

### 针对场景和射线的参数化
这里主要有如下几个改进点：

1. 借鉴NDC的思路，对射线的采样时使用逆深度（视差）的均匀采样，这样的好处是远距离的采样策略更加的合适（远处的场景视差小，相比于近处的场景，一个像素所代表的距离更远）；

2. 