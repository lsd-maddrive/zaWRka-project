#!/usr/bin/env python2
import cv2
import os
import numpy as np
from mad_detector.detector import RFSignsDetector

class LightColorDetector(object):
    def get_light(self, img, box):
        x, y, w, h = box
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        _, g, r = cv2.split(img)
        up_r = r[int(y + h/4 - h * 0.33/2):int(y + h/4 + h * 0.33/2), int(x + w/6):int(x+5*w/6)]
        down_g = g[int(y + 3*h/4 - h * 0.33/2):int(y + 3*h/4 + h * 0.33/2), int(x + w/6):int(x+5*w/6)]
        
        w_up, h_up = np.shape(up_r)
        average_up_r = np.sum(up_r)/(w_up*h_up)
        # w_down, h_down = np.shape(down_r)
        # average_down_r = np.sum(down_r)/(w_down*h_down)
        w_down, h_down = np.shape(down_g)
        average_down_g = np.sum(down_g)/(w_down*h_down)
        # print(average_up_r, average_down_g)

        # while True:
        #     cv2.imshow("color", img)
        #     cv2.imshow("UP", up_r)
        #     cv2.imshow("DOWN", down_g)
        #     if cv2.waitKey(1) == 27:
        #         break

        if (average_up_r > average_down_g):
            # print("RED")
            return "RED"
        else:
            # print("GREEN")
            return "GREEN"


def get_images_from_directory(dirpath):
    fpaths = [os.path.join(dirpath, fname) 
                for fname in os.listdir(dirpath) 
                    if fname.split('.')[-1].lower() in ['png', 'jpg', 'jpeg']]

    return fpaths

if __name__ == '__main__':
    import rospy
    rospy.init_node('test_node')
    
    # Get args 
    model_path = rospy.get_param('~model_path')
    input_path = rospy.get_param('~input')

    # model_path = "C:\\Users\\User\\Downloads\\Yolov4Tiny_mad_model.pth"
    # # input_path = "C:\\Users\\User\\Desktop\\opencv\\RF20_test\\Images"
    # input_path = "C:\\Users\\User\\Desktop\\opencv\\test_simulator"
    
    RESULT_DIRECTORY = os.path.join(input_path, 'predicted')
    
    try:
        os.makedirs(RESULT_DIRECTORY)
    except:
        pass
    
    # Execute
    det = RFSignsDetector(model_path)
    im_fpaths = get_images_from_directory(input_path)
    
    for im_fpath in im_fpaths:
        print(im_fpath)
        img = cv2.imread(im_fpath)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Resize to minimal 480
        TARGET_MIN_SIDE = 480.
        rsz_ratio = TARGET_MIN_SIDE/min(img.shape[:2])
        img = cv2.resize(img, None, fx=rsz_ratio, fy=rsz_ratio)
        
        # Predict (can be any size of input)
        bboxes, labels, scores = det.find_signs(img)

        trafficLightFound = False
        lightColorDetector = LightColorDetector()

        # Render boxes
        for i_p, (x, y, w, h) in enumerate(bboxes):
            label = labels[i_p]
            score = str(round(scores[i_p], 2))
            if label != "traffic_light":
                continue

            trafficLightFound = True
            trafficLightColor = lightColorDetector.get_light(img, (x, y, w, h))
            
            cv2.rectangle(
                img,
                (int(x), int(y)),
                (int(x+w), int(y+h)),
                color=(0, 0, 255),
                thickness=2
            )
            
            font_sz = 0.7
            font_width = 1
            cv2.putText(
                img, 
                text='{}'.format(trafficLightColor),  
                org=(int(x), int(y-5)),  
                fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                fontScale=font_sz,
                color=(0, 0, 255),
                lineType=cv2.LINE_AA, 
                thickness=font_width)

        if trafficLightFound:
            result_img_fpath = os.path.join(RESULT_DIRECTORY, os.path.basename(im_fpath))        
            # To BGR for saving
            cv2.imwrite(result_img_fpath, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
