# -*- coding: utf-8 -*-

'''
Biblioteca de funções para a visão computacional do PyArm 

Desenvolvido por membros e voluntários da RAS UFCG

Colaboradores:
    - Lyang Leme de Medeiros
    -

Descrição do branch:
    - Alterar os código para que possa ser usado em ambiente ROS

'''

import cv2
import numpy as np
import os

def get_frame(video_source=0):
    
    '''
    -
    '''

    cap = cv2.VideoCapture(video_source)

    if (cap.isOpened()== False): 
        raise ValueError("Unable to open video source", video_source)
    ret, frame = cap.read()
    
    if ret:
        return frame
    else: 
        raise ValueError("Unable to get frame")

def load_masks():
    
    '''
    -
    '''

    file = open('masks.txt', 'r')
    masks = dict()
    for line in file.readlines():
        tag, value = line.split()[0], line.split()[1:]
        tag = tag[:-1]
        new_values = []
        
        for x in value:
            x = x.strip("[],")
            new_values.append(int(x))
            masks[tag] = new_values
            
    return masks

def apply_mask(frame, color, masks):

    '''
    -
    '''

    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    
    upper = np.array(masks[color][0:3])
    lower = np.array(masks[color][3:6])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    edroded = cv2.erode(mask, kernel, iterations=2)
    dilated = cv2.dilate(edroded, kernel, iterations=2)

    res = cv2.bitwise_and(frame, frame, mask=dilated)

    return res, mask

def get_points(res, mask):

    '''
    -
    '''
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    for contour in contours:
            area = cv2.contourArea(contour)
            M = cv2.moments(contour)
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                centroid_str = '(' + str(cx) + ',' + str(cy) + ')'
     
    # Aplica delimitação de para o tamanho das áreas
    if area > 200 and area < 60000:
        return cx, cy, 30


if __name__ == '__main__':
    masks = load_masks()
    frame = get_frame()
    res, mask = apply_mask(frame, 'blue', masks)

    x,y,z = get_points(res, mask)

    print('Ponto x:' + str(x))
    print('Ponto y:' + str(y))
    