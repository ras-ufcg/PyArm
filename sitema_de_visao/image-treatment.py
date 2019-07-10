from tkinter import *
import cv2
import PIL.Image
import PIL.ImageTk

window = Tk()
canvasConteiner = Canvas(window, width=450, height=450)
canvasConteiner.pack()

def loop():
    frame = getFrame()
    update(frame)
    pass

def update(img):
    canvasConteiner.create_image(0, 0, image=img, anchor=NW)
    pass

def PILL2tkCompatible(frame):
    tkCompatibleImg = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(frame))
    return tkCompatibleImg

def getFrame():
    vid = cv2.VideoCapture(0)
    if vid.isOpened():
        get, frame = vid.read()
        if get:
            frame = PILL2tkCompatible(frame)
            return frame
    else: 
        raise ValueError("Unable to open video source", 0)

    pass


frame = getFrame()
update(frame)
# loop()
window.mainloop()

