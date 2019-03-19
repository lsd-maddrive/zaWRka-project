#!/usr/bin/env python

import cv2
import numpy as np

print(cv2.__version__)


def normalize_ycrcb(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    ycrcb[:,:,0] = cv2.equalizeHist(ycrcb[:,:,0])

    img = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
    return img

def test(img_fpath):

    img = cv2.imread(img_fpath)
    # img = normalize_ycrcb(img)
    img = cv2.bilateralFilter(img, 9, 75, 75)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)

    cv2.imshow('orig', img)
    # cv2.imshow('gray', gray)
    # cv2.imshow('lab', lab[:,:,0])
    # cv2.imshow('ycrcb', ycrcb[:,:,0])

    best_gray = lab[:,:,0]

    edges = cv2.Canny(best_gray, 100, 200)

    ret, thresh = cv2.threshold(best_gray, 190, 255, cv2.THRESH_BINARY)

    # lines = cv2.HoughLines(edges,1,np.pi/180,200)
    minLineLength = 20
    maxLineGap = 20

    img_render = img.copy()

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20, minLineLength, maxLineGap)
    for x1,y1,x2,y2 in lines[0]:
        cv2.line(img_render,(x1,y1),(x2,y2),(0, 255, 0),5)

    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def bgr_2_Y(bgr_pixel):
        return 0.114 * bgr_pixel[0] + 0.587 * bgr_pixel[1] + 0.299 * bgr_pixel[2]

    def bgr_2_Cr(bgr_pixel):
        Y = bgr_2_Y(bgr_pixel)
        return 0.713 * (bgr_pixel[2] - Y) + 128

    # Get mean colors inside contours
    for i in xrange(0, len(contours)):
        mask = np.zeros(best_gray.shape, np.uint8)
        cv2.drawContours(mask, contours, i, 255, -1)

        contour_mean_clr = cv2.mean(img, mask)

        # if bgr_2_Y(contour_mean_clr) > 210:
            # print(bgr_2_Y(contour_mean_clr))

            # cv2.drawContours(img_render, contours, i, (255, 0, 0), 2)

        if bgr_2_Cr(contour_mean_clr) > 130:
            print(bgr_2_Cr(contour_mean_clr))
            cv2.drawContours(img_render, contours, i, (0, 0, 255), 2)

    cv2.imshow('edges', edges)
    cv2.imshow('thresh', thresh)
    cv2.imshow('result', img_render)
    cv2.waitKey(0)


if __name__ == '__main__':
    test('debug/lane_debug.png')

