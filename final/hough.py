# -*- coding: utf-8 -*-

import cv2
import numpy as np
def detectCircle(file2detect):
    img = cv2.imread(file2detect,1)
    img_thr = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #1.color threshold in HSV
    lower_orange = np.array([10,130,190])        #([10,100,100])
    upper_orange = np.array([100,255,255])       #([60,255,255])
    mask = cv2.inRange(img_thr, lower_orange, upper_orange)
    res = cv2.bitwise_and(img,img, mask= mask)
    #cv2.imshow('res',res)
    #2.median blur
    res = cv2.medianBlur(res,5)
    #3.edge detection using Canny
    #edges = cv2.Canny(res,100,200)
    #cv2.imshow('edge',edges)
    #3.hough detect circle return x y r
    img = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,20,
                                param1=50,param2=10,minRadius=0,maxRadius=0)     # 修改参数精度变化
    if circles != None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(res,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(res,(i[0],i[1]),2,(0,0,255),3)
    
    cv2.imshow('detected circles',res)
    cv2.waitKey(5)
    return circles

# def Image2World(circles):
#     x = 320-circles[0][0][0]
#     y = 240-circles[0][0][1]     # y>0
#     f = 606          # 测试数据606，标定数据560.88
#     Y = 0.523      # 蹲下0.5，直立0.523     
#     X = -f*Y/y    
#     Y = -x*X/f
    
#     world = np.array([X,Y])
#     return world


if __name__ == '__main__':
    circles = detectCircle('balltest.jpg')
    print circles
    world = Image2World(circles)
    print world
