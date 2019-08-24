# yolo

1. yolo2的边界框聚类是如何做的

	yolo2对voc和coco的边界框进行了聚类，主要的做法是：
    - a. 先保存所有的ground truth位置，[xi, yi, wi, hi];
    - b. 随机初始化k个聚类中心[wj, hj];
    - c. 在每个中心[xi, yi]处，用真实值[wi, hi]与聚类中心[wj, hj]求解IOU，用$d=1-IOU$当做度量值，挑选最近接的聚类中心[wl, hl]，并将该距离值累计；
    - d. 所有中心遍历完成之后，得到累计值sum，同时对于每一个聚类中心[wk, hk]，其有一堆真实的值[wi, hi]，使用他们的平均值作为新的聚类中心，直到聚类中心变化不大为止；

	refer: [https://blog.csdn.net/shiheyingzhe/article/details/83995213](https://blog.csdn.net/shiheyingzhe/article/details/83995213)