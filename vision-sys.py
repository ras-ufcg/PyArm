## vision-sys.py 
# version 0.2

from Tkinter import * 
import cv2
import PIL.Image, PIL.ImageTk
import time

xpos = 10
ypos = 500
xspace = 40
yspace = -10
degree = 50
align = -5
align2 = -10

class App:
    def __init__(self, window, window_title, video_source):

        self.window = window
        self.window.title(window_title)
        self.video_source = video_source

        self.resize_factor = 0.5

        self.window.geometry('650x300+600+200')
        self.vid = MyVideoCapture(self.video_source)

        self.canvas = Canvas(window, width = self.vid.width*(self.resize_factor) , height = self.vid.height*(self.resize_factor))
        self.canvas.grid(row=0,column=0)

        self.canvas2 = Canvas(window, width = self.vid.width*(self.resize_factor) , height = self.vid.height*(self.resize_factor))
        self.canvas2.grid(row=0,column=1)

        self.delay = 15
        self.update()

        # H value sliders
        '''        self.HMax_label = Label(text = 'HMax').place(x=xpos,y=ypos) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+yspace)
        self.HMax_label = Label(text = 'HMin').place(x=xpos,y=ypos+degree) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+yspace+degree)

        # S value sliders
        self.HMax_label = Label(text = 'SMax').place(x=xpos,y=ypos+degree*2) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+yspace+degree*2)
        self.HMax_label = Label(text = 'SMin').place(x=xpos,y=ypos+degree*3) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+yspace+degree*3)

        # V value sliders
        self.HMax_label = Label(text = 'VMax').place(x=xpos,y=ypos+degree*4) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+degree*4-10)
        self.HMax_label = Label(text = 'VMin').place(x=xpos,y=ypos+degree*5) 
        self.slider = Scale(self.window, orient=HORIZONTAL, length=300, from_=0, to=255).place(x=xpos+xspace,y=ypos+yspace+degree*5)'''


        self.window.mainloop()

    def update(self):
        # Get a frame from the video source
        ret, frame = self.vid.get_frame()
        frame = self.vid.rescale_frame(ret,frame,self.resize_factor)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if ret:
            self.photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(frame))
            self.canvas.create_image(0, 0, image = self.photo, anchor = NW)
            self.photo2 = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(hsv))
            self.canvas2.create_image(0, 0, image = self.photo2, anchor = NW)

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