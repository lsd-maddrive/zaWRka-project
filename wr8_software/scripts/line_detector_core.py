#!/usr/bin/env python
import time
import math
import cv2
import numpy as np


class LineDetector(object):    
    def __init__(self, __angle_threshold = math.pi/8, __similarity_threshold = 25):
        self.__angle_threshold = __angle_threshold
        self.__similarity_threshold = __similarity_threshold
    
    @staticmethod
    def __convert(image):
        return image

    def process(self, image):
        frame = LineDetector.__convert(image)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('image', frame_gray)
        frame_bin = cv2.adaptiveThreshold(frame_gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,4)
        kernel = np.ones((2,2),np.uint8)
        frame_erode = cv2.erode(frame_bin,kernel)
        frame_edges = cv2.Canny(frame_erode, threshold1=50, threshold2=150, apertureSize=3, L2gradient=False)
        lines_gray= np.copy(frame)
        lines = cv2.HoughLines(frame_edges, 1, np.pi / 180, 150, None, 0, 0)
        # cv2.imshow('bin', frame_bin)
        # cv2.imshow('erode', frame_erode)
        # cv2.imshow('Canny', frame_edges)
        added_lines = []
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                if abs(theta - math.pi / 2) > math.pi / self.__angle_threshold:
                    continue

                continue_outer_loop = False
                for (rho1, theta1) in added_lines:
                    if abs(rho - rho1) + abs(theta - theta1) < self.__similarity_threshold:
                        continue_outer_loop = True
                        break
                if continue_outer_loop:
                    continue

                # a = math.cos(theta)
                # b = math.sin(theta)
                # x0 = a * rho
                # y0 = b * rho
                # pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                # pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                # cv2.line(lines_gray, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
                added_lines.append((rho, theta))
        
        # cv2.imshow('Lines', lines_gray)
        return len(added_lines) >= 2


if __name__ == "__main__":
    line_detector = LineDetector()
    cap = cv2.VideoCapture(0)
    while(1):
        _, frame = cap.read()
        print(line_detector.process(frame))
        k = cv2.waitKey(5) & 0xFF

        if k == 27:
            break
    # WAITING_PERIOD = 1
    # while not rospy.is_shutdown():
    #     result = line_detector.process("some_image")
    #     print "result is ", result
    #     time.sleep(WAITING_PERIOD)
