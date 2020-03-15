#!/usr/bin/env python
import sys
import time
import math
import cv2
import numpy as np


class LineDetector(object):
    IMG_TOP = 0.618
    IMG_BOT = 1
    def __init__(self, __min_theta = math.pi*7/16, __max_theta = math.pi*9/16,\
                 __similarity_threshold = 25, __draw_lines = False):
        self.__min_theta = __min_theta
        self.__max_theta = __max_theta
        self.__similarity_threshold = __similarity_threshold
        self.__draw_lines = __draw_lines


    def process(self, img):
        """
        Return lines and image with lines
        """
        frame = img[int(LineDetector.IMG_TOP * len(img)) : int(LineDetector.IMG_BOT * len(img))]
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_bin = cv2.adaptiveThreshold(frame_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,4)
        kernel = np.ones((3,3),np.uint8)
        frame_erode = cv2.erode(frame_bin,kernel)
        frame_edges = cv2.Canny(frame_erode, threshold1=50, threshold2=150, apertureSize=3, L2gradient=False)
        lines = cv2.HoughLines(frame_edges, 1, np.pi / 180, 150, None, 0, 0, self.__min_theta, self.__max_theta)

        added_lines = []

        #cv2.imshow('img', frame_gray)
        #cv2.imshow('bin', frame_bin)
        #cv2.imshow('erode', frame_erode)
        #cv2.imshow('Canny', frame_edges)

        lines_gray = np.copy(frame) if self.__draw_lines == True else None
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]

                continue_outer_loop = False
                for (rho1, theta1) in added_lines:
                    if abs(rho - rho1) < self.__similarity_threshold:
                        continue_outer_loop = True
                        break
                if continue_outer_loop:
                    continue

                added_lines.append((rho, theta))

                if self.__draw_lines == True:
                    a = math.cos(theta)
                    b = math.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv2.line(lines_gray, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        cv2.imshow('Lines', lines_gray)
        return added_lines, lines_gray


if __name__ == "__main__":
    """
    If argv[1] is None - use video capture
    If argv[1] is image path - use this image
    """
    line_detector = LineDetector(math.pi*7/16, math.pi*9/16, 25, True)

    if len(sys.argv) == 2:
        frame = cv2.imread(sys.argv[1], cv2.COLOR_BGR2GRAY)
    else:
        cap = cv2.VideoCapture(0)

    while(1):
        if len(sys.argv) == 1:
            _, frame = cap.read()

        print(line_detector.process(frame)[0])
        k = cv2.waitKey(5) & 0xFF

        if k == 27:
            break
