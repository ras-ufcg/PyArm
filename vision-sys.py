## vision-sys.py - v0.4
# -*- coding: utf-8 -*-

''' Colaboradores

- Lyang Leme de Medeiros
-

'''

''' Características da Versão (Beta)

- Versão: 0.4
<<<<<<< HEAD
- Marcação de objetos com caixas delimitadoras
- Exibição de seus respectivos centróides na tela de Real Time Tracking
- Atribuição de Ids parara cada objeto
- 
=======
- Rastreamento de únicos (vários objetos de cores diferentes)
    - Aplicar rotinas de diminuição de ruído para melhorar máscaras
    - Encontrar contornos nas máscaras definidas
    - Encontar o maior contorno definido
    - Calcular o centróide do contorno
    - Traçar uma caixa de marcação 
    - Exibição de seus respectivos centróides na tela de Real Time Tracking
>>>>>>> real-time-tracking

'''

from Tkinter import * 
import cv2
import PIL.Image, PIL.ImageTk
import time
import numpy as np

class App:
    def __init__(self, window, window_title, video_source=0):

        ### Window Settings ###

        self.window = window
        self.window.title(window_title)
        self.video_source = video_source
        self.window.geometry('750x600+600+200')

        ### Aux Variables ###

        self.delay = 15
        self.hmax = DoubleVar()
        self.hmin = DoubleVar()
        self.smax = DoubleVar()
        self.smin = DoubleVar()
        self.vmax = DoubleVar()
        self.vmin = DoubleVar()
        self.resize_factor = 0.55
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
        # Colors Spaces Conversions
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Variables to set color mask
        upper = np.array([self.hmax.get(), self.smax.get(), self.vmax.get()])
        lower = np.array([self.hmin.get(), self.smin.get(), self.vmin.get()])
        # Mask
        mask = cv2.inRange(hsv, lower, upper)
        # Applying Mask
        res = cv2.bitwise_and(frame, frame, mask = mask)
        # Convert frame to canvas obj
        if ret:
            self.photo_rgb = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(rgb))
            self.canvas_rgb.create_image(0, 0, image = self.photo_rgb, anchor = NW)
            self.photo_hsv = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(cv2.cvtColor(res, cv2.COLOR_BGR2RGB)))
            self.canvas_hsv.create_image(0, 0, image = self.photo_hsv, anchor = NW)
        # Loop Callback
        self.window.after(self.delay, self.update)

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
 

App(Tk(), "Tkinter and OpenCV", 1)