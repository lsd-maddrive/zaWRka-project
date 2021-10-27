#!/usr/bin/env python
import time
import cv2
import math
import numpy as np
from skimage.morphology import remove_small_objects
import matplotlib.pyplot as plt

TIME = 0
TRIGGER = False
POINTS = []

class Detector:
    def __init__(self, height, width, is_live=False):
        self.is_live = is_live

        self.a = [257, 410]
        self.b = [741, 697]
        self.c = [1131, 484]
        self.d = [585, 367]

        SRC = np.float32([self.a, self.b, self.c, self.d])

        if self.is_live:
            SRC = np.float32(
                [[257, 410],
                 [741, 697],
                 [1131, 484],
                 [585, 367]])

        self.height, self.width = height, width

        DST = np.float32(
            [[(self.width - 716) / 2, 0],
             [(self.width - 716) / 2, self.height],
             [self.width - 282, self.height],
             [self.width - 282, 0]]
        )

        self.transform = cv2.getPerspectiveTransform(SRC, DST)

        self.CAM_MATRIX = np.array(
            [[0, 0, 0],
             [0, 0, 0],
             [0, 0, 0]])

        self.DIST_COEF = np.array([0,0,0,0,0])

        self.MAIN_COLOR = (0, 255, 255)

        self.THICKNESS = 3

        self.RADIUS = 5

    def get_contours(self, eroded, min_small_area=512, area_filter=250):
        contours_coords = []

        threshold = eroded / 255.0
        threshold[threshold >= 0.1] = 1
        threshold[threshold < 0.1] = 0

        arr = threshold > 0
        threshold = remove_small_objects(arr, min_size=min_small_area, connectivity=1)
        threshold = np.array(threshold * 255, dtype=np.uint8)
        cv2.imshow('thrsh', threshold)
        contours, hierarchy = cv2.findContours(threshold, 1, 2)  

        contours_sorted_by_area = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        for k, contour_sort in enumerate(contours_sorted_by_area):
            if 0 in contour_sort or cv2.contourArea(contour_sort) < area_filter:
                continue
            x, y, w, h = cv2.boundingRect(contour_sort)
            contours_coords.append([x, y, w, h])
        #cv2.imshow('eroded', eroded)
        #cv2.waitKey(0)

        return sorted(contours_coords, key=lambda x: x[1])

    def detection_by_frame(self, frame):

        control_x = 312    
        control_y = 443    

        undistorted_image = frame

        if self.is_live:
            undistorted_image = cv2.undistort(frame, self.CAM_MATRIX, self.DIST_COEF)  
            cv2.imshow('undistorted_image', undistorted_image)

        #cv2.circle(undistorted_image, tuple(a), 3, (0, 0, 255), 3)
        #cv2.circle(undistorted_image, tuple(b), 3, (0, 0, 255), 3)
        #cv2.circle(undistorted_image, tuple(c), 3, (0, 0, 255), 3)
        #cv2.circle(undistorted_image, tuple(d), 3, (0, 0, 255), 3)

        #cv2.namedWindow("viewed", cv2.WINDOW_NORMAL)
        #cv2.imshow('viewed', undistorted_image)
        #cv2.waitKey(0)

        viewed = cv2.warpPerspective(undistorted_image, self.transform, (self.width, self.height),
                                     flags=cv2.INTER_LINEAR)   
        
        viewed = cv2.resize(viewed, dsize=(int(self.width / 2), int(self.height / 2)), interpolation=cv2.INTER_LINEAR)

        #cv2.imshow('viewed', viewed)
        #cv2.waitKey(0)
       
        #cv2.circle(viewed, (control_x, control_y), self.RADIUS, self.MAIN_COLOR, self.THICKNESS)

        # filters
        viewed = cv2.GaussianBlur(viewed, ksize=(5, 5), sigmaX=0)

        result = {
            'found': False,
            'time': None,
            'parking_params': {
                'x': None,
                'y': None,
                'center': None
            },
            'image': viewed
        }

        gray_y = cv2.cvtColor(viewed, cv2.COLOR_BGR2HSV) 

        # split by x & y
        gray_y = gray_y[:, :, 2]
        gray_x = gray_y.copy()

        # split kernels
        line_y = np.zeros((11, 11), dtype=np.uint8)
        line_y[5, ...] = 1
        line_x = np.transpose(line_y)

        gray_y -= cv2.morphologyEx(gray_y, cv2.MORPH_OPEN, line_y, iterations=3)
        gray_x -= cv2.morphologyEx(gray_x, cv2.MORPH_OPEN, line_x, iterations=3)

        kernel_x = np.ones((5, 40), dtype=np.uint8)
        kernel_y = np.ones((10, 1), dtype=np.uint8)

        dilated_view_x = cv2.dilate(gray_x, kernel_x, iterations=2)
        kernel_x = np.ones((5, 40), dtype=np.uint8)
        eroded_view_x = cv2.erode(dilated_view_x, kernel_x, iterations=2)
        eroded_view_y = cv2.erode(gray_y, kernel_y, iterations=2)

        #cv2.imshow('gray_x', gray_x)
        #cv2.imshow('gray_y', gray_y)
        #cv2.imshow('eroded_x', eroded_view_x)
        #cv2.imshow('eroded_y', eroded_view_y)
        #cv2.waitKey(0)

        # get contours for next analyse
        contours_x_sorted = self.get_contours(eroded_view_x)
        contours_y_sorted = self.get_contours(eroded_view_y)

        # FILTERING RAGE
        if len(contours_x_sorted) > 1 and len(contours_y_sorted) > 0:
            x1, y1, w1, h1 = contours_x_sorted[-1]
            x2, y2, w2, h2 = contours_x_sorted[-2]
            cv2.rectangle(viewed, (x1, y1), (x1+w1, y1+h1), (0, 0, 255))
            cv2.rectangle(viewed, (x2, y2), (x2+w2, y2+h2), (0, 0, 255))

            # finding middle point of entrance            
            middle_point_x = int((x2 + x1) / 2)
            middle_point_y = int((y2 + h2 + y1) / 2)

            # filter out parking lot that placed lower than control point
            if middle_point_y >= control_y:
                return result

            for contour_y in contours_y_sorted:
                b_x, b_y, b_w, b_h = contour_y

                # finding middle point of vertical line
                back_middle_x = int((2 * b_x + b_w) / 2)
                back_middle_y = int((2 * b_y + b_h) / 2)

                # check if it is suitable vertical line or not
                if b_y < middle_point_y < b_y + b_h:
                    
                    # check the parking lot area 
                    area = (b_x - x1) * (y1 - y2 - h2)
                    if 40000 < area < 70000:
                        center_point_x, center_point_y = int((b_x + middle_point_x) / 2), int(
                            (middle_point_y + back_middle_y) / 2)

                        delta_x = (control_y - center_point_y) * 0.002357
                        delta_y = (center_point_x - control_x) * 0.002321

                        cv2.line(viewed, (middle_point_x, middle_point_y),
                                    (int(b_x + b_w / 2), int(b_y + b_h / 2)),
                                    self.MAIN_COLOR, self.THICKNESS)
                        cv2.circle(viewed, (back_middle_x, back_middle_y), self.RADIUS, self.MAIN_COLOR,
                                    self.THICKNESS)
                        cv2.circle(viewed, (center_point_x, center_point_y), self.RADIUS, self.MAIN_COLOR,
                                    self.THICKNESS)
                        cv2.line(viewed, (x1, y1), (x2, y2 + h2), self.MAIN_COLOR, self.THICKNESS)
                        cv2.circle(viewed, (x1, y1), self.RADIUS, self.MAIN_COLOR, self.THICKNESS)
                        cv2.circle(viewed, (x2, y2 + h2), self.RADIUS, self.MAIN_COLOR, self.THICKNESS)

                        result['found'] = True
                        result['time'] = time.time()
                        result['image'] = viewed
                        result['parking_params']['x'] = delta_x
                        result['parking_params']['y'] = delta_y
                        result['parking_params']['center'] = center_point_x


        return result


if __name__ == '__main__':
    #cap = cv2.VideoCapture('../output.avi')
    #height, width = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)), int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    img = cv2.imread('/home/artemon12/PycharmProjects/parking_diploma/pics/lot_sim.jpg')

    height, width, channel = img.shape
    detector = Detector(height=height, width=width, is_live=False)

    while True:
        #ret, frame = cap.read()
        ret = True
        frame = img
        if ret is False:
            print('Videostream not found')
            break

        start = time.time()
        data = detector.detection_by_frame(frame)
        end = time.time()
        ms = round(end - start, 4)

        result = data['image']

        if data['found'] and TRIGGER is not True:
            if TIME is 0:
                TIME = data['time']

            POINTS.append(data['parking_params']['center'])

            if data['time'] - TIME > 1.5:
                average = np.std(POINTS)
                if average < 5:
                    TRIGGER = True

        if TRIGGER:
            cv2.putText(result, 'something found!', (50, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                        (255, 255, 255))

        cv2.putText(result, 'time ' + str(ms) + ' ms', (50, 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))

        cv2.imshow('result', result)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    #cap.release()
    cv2.destroyAllWindows()
