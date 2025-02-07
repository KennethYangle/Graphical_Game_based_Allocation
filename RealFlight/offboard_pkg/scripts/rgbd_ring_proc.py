#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from scipy.optimize import leastsq



def color_red_proc(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    chH,chS,chV = cv2.split(hsv_img)
    chH[chH < 60] += 180
    hsv_img = cv2.merge([chH,chS,chV])
    
    # 亦庄红
    # low_L = [172, 58, 123]
    # up_L  = [182, 227, 255]

    # 上海红
    low_L = [160, 115, 0]
    up_L  = [190, 255, 255]
    img_bin = cv2.inRange(hsv_img, np.array(low_L), np.array(up_L))
    return img_bin



# 计算mask内的平均深度
def depth_vote_proc(depth_float,mask,dis_bias = 0.75):
    depth_res =  depth_float.copy()
    hist,bin_edges  = np.histogram(depth_float[mask],bins=20,range=(0,10))
    id_max = np.argmax(hist[1:])+1
    dis_mid = bin_edges[id_max]+0.25
    depth_mask1 = (depth_float > dis_mid+dis_bias)
    depth_mask2 = (depth_float < dis_mid-dis_bias)
    depth_res[depth_mask2] = 0
    depth_res[depth_mask1] = 0
    mask_available = (depth_res > 0)
    depth_real = np.average(depth_res[mask_available])
    return depth_real


# 最小二乘拟合圆误差函数
def residuals(p, xL, yL):
    xc,yc,r = p
    rL = [ np.linalg.norm([x-xc,y-yc]) for x,y in zip(xL, yL) ]
    r_aver = np.average(rL)
    r_err = np.array([
        (i-r_aver)**2 + (i-r)**2
        for i in rL
    ])
    return r_err






def multi_ring_dect(color_img,depth_img):
    img_res = color_img

    # 颜色图处理
    color_bin = color_red_proc(color_img)
    # 开运算,消除小的物体
    kernel = np.ones(shape=[2, 2], dtype=np.uint8)
    open_result = cv2.morphologyEx(color_bin, op=cv2.MORPH_OPEN, kernel=kernel, iterations=2)

    # 深度图像预处理
    depth_mf = cv2.medianBlur(depth_img, 5)
    depth_beyond_mask = (depth_mf > 10000)
    depth_float = depth_mf/1000
    depth_float[depth_beyond_mask] = 0

    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(open_result, connectivity=4)

    circleL = []


    for i in range(1,num_labels):
        # 太小的跳过
        if stats[i][-1] < 2000:
            continue

        cx,cy = centroids[i]
        mask = (labels == i)
        yxL = np.where(mask == True)
        xL_np = yxL[1] - cx
        yL_np = yxL[0] - cy


        thetaL = [np.arctan2(y,x) for x,y in zip(xL_np,yL_np)]

        hist,_ = np.histogram(thetaL,bins=24,range=(-np.pi,np.pi))
        
        radiusL = [ x**2 + y**2 for x,y in zip(xL_np,yL_np)]
        r = np.sqrt(np.average(radiusL))
        
        # 完整的圆直接用计算值
        # 缺少45-60是不完整的圆进行拟合
        if np.sum(hist==0) > 3:
            pars = [cx,cy,r]
            cx,cy,r = leastsq(residuals, pars, args=(yxL[1],yxL[0]))[0]

        real_dis = depth_vote_proc(depth_float,mask,dis_bias= 0.75)
        circleL.append([cx,cy,r,real_dis])

        cv2.circle(img_res, (int(cx), int(cy)), 3, (255, 255, 255), -1)
        cv2.circle(img_res, (int(cx), int(cy)), int(r), (255, 255, 255), 2)
    
    return circleL



def single_centre_ring_dect(color_img,depth_img,roi_w=200):
    img_res = color_img

    # 颜色图处理
    color_roi = np.zeros_like(depth_img,dtype=np.uint8)
    color_roi[:,320-roi_w:320+roi_w] = 1
    color_bin = color_red_proc(color_img)*color_roi
    color_mask = (color_bin > 0)



    # 深度图像预处理
    depth_mf = cv2.medianBlur(depth_img, 5)
    depth_beyond_mask = (depth_mf > 10000)
    depth_float = depth_mf/1000
    depth_float[depth_beyond_mask] = 0

    real_dis = depth_vote_proc(depth_float,color_mask,dis_bias= 0.75)

    img_mask = color_mask
    img_bin = color_bin
    # 形态学操作
    kernel_size_unit = np.clip(np.sqrt(np.sum(img_mask))/480*1.5, 0.5, 2)
    kernel = np.ones(shape=[int(kernel_size_unit*3), int(kernel_size_unit*4)], dtype=np.uint8)
    open_result = cv2.morphologyEx(img_bin, op=cv2.MORPH_OPEN, kernel=kernel, iterations=2)
    kernel = np.ones(shape=[int(kernel_size_unit*3*2), int(kernel_size_unit*4*2)], dtype=np.uint8)
    close_result = cv2.morphologyEx(open_result, op=cv2.MORPH_CLOSE, kernel=kernel, iterations=4)

    mm = cv2.moments(close_result)
    if mm['m00'] < 400:
        return [-1]*4

    yxL = np.where(close_result > 0 )
    cx = mm['m10'] / mm['m00']
    cy = mm['m01'] / mm['m00']
    radiusL = [ (cx-x)**2 + (cy-y)**2 for y,x in zip(yxL[0],yxL[1])]
    r = np.sqrt(np.average(radiusL))
    
    img_res = color_img
    cv2.circle(img_res, (int(cx), int(cy)), 3, (255, 255, 255), -1)
    cv2.circle(img_res, (int(cx), int(cy)), int(r), (255, 255, 255), 2)

    return cx,cy,r,real_dis




if __name__ == '__main__':
    color_path = "data/2022-12-06-18-04-50/00010.png"
    color_img = cv2.imread(color_path)

    depth_path = "data/2022-12-06-18-04-50/00010.npy"
    depth_img= np.load(depth_path)
    circleL = multi_ring_dect(color_img,depth_img)
    print(circleL)

    cv2.imshow("result",color_img)
    cv2.waitKey(0)