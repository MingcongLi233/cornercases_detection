import random
from matplotlib import pyplot as plt
import numpy as np
import math
import open3d 

def ground_segmentation(data,iters):
    
    """
    Ground removal by RANSAC

    """
    idx_segmented = []
    segmented_cloud = []
    iters = iters
    sigma = 0.4   #ToDo:Threshold
    #Best model: aX + bY + cZ +D= 0
    best_a = 0
    best_b = 0
    best_c = 0
    best_d = 0
    pretotal = 0 #上一次inline的点数
    #希望的到正确模型的概率
    P = 0.99
    n = len(data)    #点的数目
    outline_ratio = 0.6   #ToDo: e
    for i in range(iters):
        ground_cloud = []
        idx_ground = []
        #step1 选择可以估计出模型的最小数据集，对于平面拟合来说，就是三个点
        sample_index = random.sample(range(n),3)    #重数据集中随机选取3个点
        point1 = data[sample_index[0]]
        point2 = data[sample_index[1]]
        point3 = data[sample_index[2]]
        #step2 求解模型
        ##先求解法向量
        point1_2 = (point1-point2)      #向量 poin1 -> point2
        point1_3 = (point1-point3)      #向量 poin1 -> point3
        N = np.cross(point1_3,point1_2)            #向量叉乘求解 平面法向量
        ##slove model 求解模型的a,b,c,d
        a = N[0]
        b = N[1]
        c = N[2]
        d = -N.dot(point1)
        #step3 将所有数据带入模型，计算出“内点”的数目；(累加在一定误差范围内的适合当前迭代推出模型的数据)
        total_inlier = 0
        pointn_1 = (data - point1)    #sample（三点）外的点 与 sample内的三点其中一点 所构成的向量
        distance = abs(pointn_1.dot(N))/ np.linalg.norm(N)     #求距离
        ##使用距离判断inline
        idx_ground = (distance <= sigma)
        total_inlier = np.sum(idx_ground == True)    #统计inline得点数
        ##判断当前的模型是否比之前估算的模型
        if total_inlier > pretotal:                                           #     log(1 - p)
            iters = math.log(1 - P) / math.log(1 - pow(total_inlier / n, 3))  #N = ------------
            pretotal = total_inlier                                               #log(1-[(1-e)**s])
            #获取最好得 abcd 模型参数
            best_a = a
            best_b = b
            best_c = c
            best_d = d

        # 判断是否当前模型已经符合超过 inline_ratio
        if total_inlier > n*(1-outline_ratio):
            break
    print("iters = %f" %iters)
    #提取分割后得点
    idx_segmented = np.logical_not(idx_ground)
    ground_cloud = data[idx_ground]
    segmented_cloud = data[idx_segmented]
    return ground_cloud,segmented_cloud


