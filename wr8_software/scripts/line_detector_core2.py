#!/usr/bin/env python2
import sys
import time
import math
import cv2
import numpy as np
import os

class LineDetector(object):
    def __init__(self,
                 img_top_cropping_border, \
                 img_bot_cropping_border, \
                 min_theta = math.pi*7/16, \
                 max_theta = math.pi*9/16, \
                 rho_min_offset = 5,
                 rho_max_offset = 10, \
                 theta_similarity_threshold = math.pi/256, \
                 min_vote = 225,
                 draw_lines = False):
        self.__img_top_cropping_border = img_top_cropping_border
        self.__img_bot_cropping_border = img_bot_cropping_border
        self.__min_theta = min_theta
        self.__max_theta = max_theta
        self.__rho_min_offset = rho_min_offset
        self.__rho_max_offset = rho_max_offset
        self.__theta_similarity_threshold = theta_similarity_threshold
        self.__min_vote = min_vote
        self.__draw_lines = draw_lines


    def process(self, img):
        """
        Return lines and image with lines
        """
        frame = img[int(self.__img_top_cropping_border * len(img)) : \
                    int(self.__img_bot_cropping_border * len(img))]
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        frame_gray = cv2.GaussianBlur(frame_gray, (5, 5), 0)
        frame_bin = cv2.adaptiveThreshold(frame_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,4)
        kernel = np.ones((3,3),np.uint8)
        #frame_erode = cv2.erode(frame_bin,kernel)
        frame_erode = cv2.morphologyEx(frame_bin, cv2.MORPH_CLOSE, kernel)
        frame_edges = cv2.Canny(frame_erode, threshold1=200, threshold2=300, apertureSize=3, L2gradient=False)
        lines = cv2.HoughLines(frame_edges, 1, np.pi / 180, self.__min_vote, None, 0, 0, self.__min_theta, self.__max_theta)
        lines_gray = np.copy(frame) if self.__draw_lines == True else None

        if lines is None:
            return [], frame_erode

        #cv2.imshow('img', frame_gray)
        #cv2.imshow('bin', frame_bin)
        #cv2.imshow('erode', frame_erode)
        #cv2.imshow('Canny', frame_edges)

        added_lines = []
        stop_condition = False
        for first_idx in range(len(lines) - 1):
            if stop_condition:
                break
            for second_idx in range(first_idx + 1, len(lines)):
                rho1 = lines[first_idx][0][0]
                theta1 = lines[first_idx][0][1]
                rho2 = lines[second_idx][0][0]
                theta2 = lines[second_idx][0][1]
                delta_rho = abs(rho1 - rho2)
                delta_theta = abs(theta1 - theta2)
                if delta_rho >= self.__rho_min_offset and \
                   delta_rho <= self.__rho_max_offset and \
                   delta_theta < self.__theta_similarity_threshold:
                    added_lines.append((rho1, theta1))
                    added_lines.append((rho2, theta2))
                    stop_condition = True
                    break

        if self.__draw_lines is True:
            for line in added_lines:
                self._draw_line(line[0], line[1], lines_gray)

        #cv2.imshow('Lines', lines_gray)
        return added_lines, lines_gray

    def _draw_line(self, rho, theta, lines_gray):
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1280*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1280*(a)))
        cv2.line(lines_gray, pt1, pt2, (255,0,0), 3, cv2.LINE_AA)


if __name__ == "__main__":
    """
    If argv[1] is None - use video capture
    If argv[1] is image path - use this image
    """
    line_detector = LineDetector(img_top_cropping_border=0.05,
                                 img_bot_cropping_border=1,
                                 min_theta=1.37,
                                 max_theta=1.77,
                                 rho_min_offset=5,
                                 rho_max_offset=10,
                                 #rho_similarity_threshold=25,
                                 theta_similarity_threshold=0.012265,
                                 min_vote=60,
                                 draw_lines=True)
    DIRPATH = sys.argv[1]
    for fname in os.listdir(DIRPATH):    
        frame = cv2.imread(os.path.join(DIRPATH, fname), cv2.COLOR_BGR2GRAY)
        lines, img = line_detector.process(frame)
        print("new pic", lines)
        cv2.imwrite(os.path.join(DIRPATH, fname+"_detected"), img)
