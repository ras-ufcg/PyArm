## vision-sys.py - v0.4
# -*- coding: utf-8 -*-

''' Colaboradores

- Lyang Leme de Medeiros
-

'''

''' Características da Versão (Beta)

- Versão: 0.4
    - Aplicar rotinas de diminuição de ruído para melhorar máscaras - OK
    - Encontrar contornos nas máscaras definidas - OK
    - Calcular o centróide do contorno e exibir no video - OK
    - Traçar uma caixa de marcação - OK

'''

from Tkinter import * 
import cv2
import PIL.Image, PIL.ImageTk
import time
import numpy as np

class App:

    ### Constructor Function ###

    def __init__(self, window, window_title, video_source=0):

        ### Window Settings ###

        self.window = window
        self.window.title(window_title)
        self.video_source = video_source
        self.window.geometry('750x600+600+200')

        ### Aux Variables ###

        self.resize_factor = 0.55
        self.delay = 15
        self.hmax = DoubleVar()
        self.hmin = DoubleVar()
        self.smax = DoubleVar()
        self.smin = DoubleVar()
        self.vmax = DoubleVar()
        self.vmin = DoubleVar()
        self.name = StringVar()

        ### Aux Objects ###

        self.vid = MyVideoCapture(self.video_source)

        ### Widgets ###

        ''' Labels '''
        self.label_RGB = Label(self.window, text = 'Real Time Tracking')
        self.label_Mask = Label(self.window, text = 'Color Mask Veiwer')

        ''' Conteiners '''
        self.canvas_rgb = Canvas(self.window, width = self.vid.width*(self.resize_factor) , height = self.vid.height*(self.resize_factor))
        self.canvas_hsv = Canvas(self.window, width = self.vid.width*(self.resize_factor) , height = self.vid.height*(self.resize_factor))
    
        ''' H value sliders '''
        self.slider_hmax = Scale(self.window, orient=HORIZONTAL, variable = self.hmax , label = 'HMax', length=300, from_=0, to=255)
        self.slider_hmax.set(255)
        self.slider_hmin = Scale(self.window, orient=HORIZONTAL, variable = self.hmin , label = 'HMin', length=300, from_=0, to=255)
       
        ''' S value sliders '''
        self.slider_smax = Scale(self.window, orient=HORIZONTAL, variable = self.smax , label = 'SMax', length=300, from_=0, to=255)
        self.slider_smax.set(255)
        self.slider_smin = Scale(self.window, orient=HORIZONTAL, variable = self.smin , label = 'SMin', length=300, from_=0, to=255)
        
        ''' V value sliders '''
        self.slider_vmax = Scale(self.window, orient=HORIZONTAL, variable = self.vmax, label = 'VMax', length=300, from_=0, to=255)
        self.slider_vmax.set(255)
        self.slider_vmin = Scale(self.window, orient=HORIZONTAL, variable = self.vmin, label = 'VMin', length=300, from_=0, to=255)
        
        ''' Misc '''
        self.name_text_box = Entry(window, textvariable=self.name)
        self.btn_save = Button(window, text='Save', command=self.save)
        self.btn_rst = Button(window, text='Reset Mask Values', command=self.rst)

        ### Posicionamento ###

        self.label_RGB.grid(row=0, column=0)
        self.label_Mask.grid(row=0, column=1)
        self.canvas_rgb.grid(row=1, column=0)
        self.canvas_hsv.grid(row=1, column=1)
        self.slider_hmax.grid(row=2, column=1)
        self.slider_hmin.grid(row=2, column=0)
        self.slider_smax.grid(row=3, column=1)
        self.slider_smin.grid(row=3, column=0)
        self.slider_vmax.grid(row=4, column=1)
        self.slider_vmin.grid(row=4, column=0)
        self.name_text_box.grid(row=5, column=0)
        self.btn_save.grid(row=6, column=0)
        self.btn_rst.grid(row=5,column=1) 
        
        ### Functions Calling ###

        self.update()

        self.window.mainloop()

    ### Class Functions ###

    def save(self):
        # construir funcao para salvar mascaras
        pass

    def rst(self):
        # construir funcao para resetar valores da mascara
        pass

    def update(self):
        # Get a frame from the video source and rescale it
        ret, frame = self.vid.get_frame()
        frame = self.vid.rescale_frame(ret,frame,self.resize_factor)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        res = self.frame_prcess(frame)

        # Convert frame to canvas obj
        if ret:
            self.photo_rgb = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(rgb))
            self.canvas_rgb.create_image(0, 0, image = self.photo_rgb, anchor = NW)
            self.photo_hsv = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(cv2.cvtColor(res, cv2.COLOR_BGR2RGB)))
            self.canvas_hsv.create_image(0, 0, image = self.photo_hsv, anchor = NW)
            
        # Loop Callback
        self.window.after(self.delay, self.update)

    def frame_prcess(self, frame):
        # Gaussian Blur on frame to reduce noise and details, it makes easyer to find especifcs contours 
        blur = cv2.GaussianBlur(frame, (5, 5), 0)

        # Colors Spaces conversions
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        # Variables to set color mask
        upper = np.array([self.hmax.get(), self.smax.get(), self.vmax.get()])
        lower = np.array([self.hmin.get(), self.smin.get(), self.vmin.get()])
        
        # Mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Applying Mask
        res = cv2.bitwise_and(frame, frame, mask = mask)

        # Fiding contours
        kernel = np.ones((5,5), np.uint8) 
        dilated = cv2.dilate(mask, kernel, iterations=1)
        im2, contours, hierarchy = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        # Drawing contours
        for contour in contours:
            area = cv2.contourArea(contour)
            M = cv2.moments(contour)
            if (M['m00'] != 0):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid_str = '('+str(cx)+','+str(cy)+')'
                
            if area > 1000 and area < 60000:
                cv2.drawContours(res, contour, -1, (150, 0, 200), 3)
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(res, [box], 0, (100,255,200), 3)
                cv2.putText(res, centroid_str, (cx+5,cy+5), cv2.FONT_HERSHEY_PLAIN, 1,(0,0,255),1,cv2.LINE_AA)
                cv2.circle(res,(cx,cy), 3, (255,0,255), -1)

        return res

class MyVideoCapture:
    def __init__(self, video_source=0):
        # Open the video source
        self.vid = cv2.VideoCapture(video_source)
        if not self.vid.isOpened():
            raise ValueError("Unable to open video source", video_source)

        # Get video source width and height
        self.width = self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def rescale_frame(self, ret, frame, percent=75):
        if ret:
            width = int(frame.shape[1] * percent)
            height = int(frame.shape[0] * percent)
            dim = (width, height)
            return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
        else: 
            return None

    def get_frame(self):
        if self.vid.isOpened():
            ret, frame = self.vid.read()
            if ret:
                # Return a boolean success flag and the current frame converted to BGR
                return (ret, frame)
            else:
                return (ret, None)
        else:
            return (ret, None)
 

App(Tk(), "VERA Vision System - v0.4", 1)