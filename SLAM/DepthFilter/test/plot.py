'''
@Author: your name
@Date: 2020-04-14 23:12:14
@LastEditTime: 2020-04-17 23:55:27
@LastEditors: Please set LastEditors
@Description: In User Settings Edit
@FilePath: /DepthFilter/test/plot.py
'''

import os
import numpy as np
import cv2

from matplotlib import pyplot as plt

if __name__ == "__main__":
    depth_image_path  = '/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/depth.png'
    depth_image_path1 = '/home/ubuntu/Projects/PaperSummary/SLAM/DepthFilter/test/build/depth2.png'

    depth  = cv2.imread(depth_image_path,  cv2.IMREAD_UNCHANGED)
    depth1 = cv2.imread(depth_image_path1, cv2.IMREAD_UNCHANGED)

    depth[depth>10] = 10

    fig, axes = plt.subplots(1, 2)

    ax0 = axes[0].imshow(depth)
    ax1 = axes[1].imshow(depth1)

    axes[0].axis('off')
    axes[1].axis('off')

    fig.colorbar(ax0, ax=axes[0])
    fig.colorbar(ax1, ax=axes[1])
    plt.show()
