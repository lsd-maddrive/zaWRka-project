#!/usr/bin/env python

import cv2
import numpy as np

def test(img_fpath):

    img = cv2.imread(img_fpath)

    blur = cv2.bilateralFilter(img, 9, 75, 75)

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    ycrcb = cv2.cvtColor(blur, cv2.COLOR_BGR2YCrCb)
    lab = cv2.cvtColor(blur, cv2.COLOR_BGR2Lab)

    cv2.imshow('orig', blur)
    # cv2.imshow('gray', gray)
    # cv2.imshow('lab', lab[:,:,0])
    # cv2.imshow('ycrcb', ycrcb[:,:,0])

    best_gray = lab[:,:,0]
    
    edges = cv2.Canny(best_gray, 100, 200)

    ret, thresh = cv2.threshold(best_gray, 190, 255, cv2.THRESH_BINARY)

    # lines = cv2.HoughLines(edges,1,np.pi/180,200)
    minLineLength = 20
    maxLineGap = 20

    
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 200, minLineLength, maxLineGap)

    img_render = img.copy()

    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img_render, contours, -1, (255,0,0), 3)

    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img_render,(x1,y1),(x2,y2),(0, 255, 0),5)

    cv2.imshow('edges', edges)
    cv2.imshow('thresh', thresh)
    cv2.imshow('result', img_render)
    cv2.waitKey(0)


if __name__ == '__main__':
    test('debug/lane_debug.png')

