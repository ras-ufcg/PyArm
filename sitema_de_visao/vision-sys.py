# -*- coding: utf-8 -*-

'''
Sistema de visão computacional para caracterização e controle de braços
robóticos baseados na plataforma Arduino.

Desenvolvido por membros e voluntários da RAS UFCG

Colaboradores:
    - Lyang Leme de Medeiros
    -

Características da versão:
    - Adicionar opção para calibração da ogrigem de coordenadas

'''

__version__ = '0.8'
__author__ = '''Lyang Leme de Medeiros'''

import tkinter as tk
from tkinter import messagebox as msgbx
from tkinter import filedialog
import cv2
import PIL.Image
import PIL.ImageTk
import numpy as np
import os


def rescale_frame(ret, frame, percent=75):
    ''' Redimenciona o quadro recebido '''

    if ret:
        width = int(frame.shape[1] * percent)
        height = int(frame.shape[0] * percent)
        dim = (width, height)
        return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    else:
        return None


def get_frame():
    ''' Obtem um novo quadro da entrada de vídeo '''
    global cam
    if cam.isOpened():
        ret, frame = cam.read()
        if ret:
            return (ret, frame)
        else:
            return (ret, None)
    else:
        return (ret, None)


def load_file():
    ''' Carrega máscaras de um arquivo '''

    path = filedialog.askopenfilename()
    file = open(path, 'r')
    for line in file.readlines():
        tag, value = line.split()[0], line.split()[1:]
        tag = tag[:-1]
        new_values = []
        for x in value:
            x = x.strip("[],")
            new_values.append(int(x))
        masks[tag] = new_values


def save_as():
    ''' Salva as mascaras da seção atual em um arquivo .txt '''

    path = filedialog.asksaveasfilename(
        initialdir=os.getcwd(),
        filetypes=(("Text", "*.txt"), ("All Files", "*.*")),
        defaultextension=".txt",
    )

    file = open(path, 'w+')
    for tag, values in masks.items():
        file.write(tag + ': ' + str(values) + '\n')
    file.close()


def show_about():
    ''' Mostra informações sobre o aplicativo '''

    msgbx.showinfo(
        title='Sobre',
        message='''Software de visão computacional desenvolvido por ''' +
        '''membros e voluntários do Capítulo Estudantil IEEE RAS UFCG.''')


def save():
    ''' Salva a os valores da máscara no dicionário da seção '''

    mask_value = [
        hmax.get(), smax.get(), vmax.get(),
        hmin.get(), smin.get(), vmin.get(),
    ]

    masks[name.get()] = mask_value
    name.set('')
    rst()


def rst():
    ''' Reseta valores das máscaras '''

    slider_hmax.set(255)
    slider_smax.set(255)
    slider_vmax.set(255)
    slider_hmin.set(0)
    slider_smin.set(0)
    slider_vmin.set(0)


def calibrate():
    ''' Coloca o centro do objeto selecionado como origem das coordenadas cartesianas '''
    global origin, new_origin

    origin = new_origin


def draw_contour(mask, res, name=''):
    ''' Encontra e desenha contorno nas áreas da resultantes da máscara '''
    global origin, new_origin
    # Procura os contornos
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Desenha os contornos
    for contour in contours:
        area = cv2.contourArea(contour)
        M = cv2.moments(contour)
        if (M['m00'] != 0):
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            new_origin = (cx, cy)
            centroid_str = '(' + str(origin[0] - cx) + ',' + str(origin[1] - cy) + ')'

        # Aplica delimitação de para o tamanho das áreas
        if area > 200 and area < 60000:
            cv2.drawContours(res, contour, -1, (150, 0, 200), 3)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(res, [box], 0, (100, 255, 200), 3)
            cv2.putText(res, centroid_str, (cx + 5, cy + 5), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)
            if name != '':
                cv2.putText(res, name, (cx + 5, cy + 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1, cv2.LINE_AA)

            # Desenha um ponto no centróide do objeto
            cv2.circle(res, (cx, cy), 3, (255, 0, 255), -1)
            # Desenha uma linha da origem até o centroide
            cv2.line(res, origin, (cx, cy), (255, 0, 0), thickness=2, lineType=8, shift=0)
    return res


def frame_prcess(frame):
    ''' Aplica a máscara de acordo com os valores dos sliders '''

    # Obtem os valores dos sliders de máximo
    upper = np.array([hmax.get(), smax.get(), vmax.get()])

    # Obtem os valores dos sliders de mínimo
    lower = np.array([hmin.get(), smin.get(), vmin.get()])

    # Aplica o o recorte ao quadro segundo os valores de máximo e mínimo
    mask = get_mask(frame, upper, lower)

    # Recorta os pixels com valores dentro do range da máscara fazendo um and bit a bit com a mask
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Desenha o contorno ao recorte
    res = draw_contour(mask, res)

    return res


def get_mask(frame, upper, lower):
    ''' Pega os valores dos sliders e aplica funções para construção da máscara '''

    # Aplica um blur gaussiano ao quadro para reduzir os detalhes e o ruído.
    # Isso deixa mais fácil encontrar contornos específicos
    blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # Conversão de space colors BGR->HSV
    # HSV é um color space que facilita na detecçã de contornos
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # Os pixels dentro do range de cor ficaram com valor 1 e os demais com valor 0
    mask = cv2.inRange(hsv, lower, upper)

    # Matriz para aplicação de métodos sobre o quadro
    kernel = np.ones((5, 5), np.uint8)

    # Aplica eroção nos pixels com valor 1
    edroded = cv2.erode(mask, kernel, iterations=1)

    # Aplica dilataçã nos pixels de valor 1
    dilated = cv2.dilate(edroded, kernel, iterations=1)

    return dilated


def update():
    ''' Pega quadros da entrada de vídeo e atualiza o App '''
    global canvas_hsv, canvas_rgb, window, resize_factor, masks, delay
    # Pega quadros da entrada de vídeo
    ret, frame = get_frame()

    # Redimensiona o quadro
    frame = rescale_frame(ret, frame, resize_factor)

    # Aplica a máscara em tempo real de acordo com os sliders
    res = frame_prcess(frame)

    # Converte o space color do quadro
    rtt = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Verifica se já existe alguma máscara no dicionário
    # e caso exista, carrwda as máscaras no quadro de rastreamento
    # em tempo real
    if len(masks) != 0:
        for tag, value in masks.items():
            mask = get_mask(frame, np.array(value[0:3]), np.array(value[3:6]))
            rtt = draw_contour(mask, rtt, tag)

    # Converte o quadro pra um objeto canvas

    if ret:
        photo_rgb = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(rtt))
        canvas_rgb.create_image(0, 0, image=photo_rgb, anchor=tk.NW)
        photo_hsv = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(cv2.cvtColor(res, cv2.COLOR_BGR2RGB)))
        canvas_hsv.create_image(0, 0, image=photo_hsv, anchor=tk.NW)
    # Loop
    window.after(delay, update)


window = tk.Tk()
window.title("SVC - RAS UFCG - v" + __version__)
window.geometry('750x600+400+100')

video_source = 0
resize_factor = 0.55
delay = 15
hmax = tk.IntVar()
hmin = tk.IntVar()
smax = tk.IntVar()
smin = tk.IntVar()
vmax = tk.IntVar()
vmin = tk.IntVar()
name = tk.StringVar()
masks = dict()
new_origin = (0, 0)
origin = (0, 0)

cam = cv2.VideoCapture(video_source)

if not cam.isOpened():
            raise ValueError("Não foi possível habilitar a entrada de vídeo: ", video_source)

# Obtem a altura e a largura do quadro capturado
width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)

# Declarações e configurações de widgets

''' Labels '''
label_RGB = tk.Label(window, text='Rasteramento em tempo real')
label_Mask = tk.Label(window, text='Visualizador de Máscaras')

''' Conteiners '''
canvas_rgb = tk.Canvas(window, width=width * (resize_factor),
                       height=height * (resize_factor))

canvas_hsv = tk.Canvas(window, width=width * (resize_factor),
                       height=height * (resize_factor))

''' H value sliders '''
slider_hmax = tk.Scale(window, orient=tk.HORIZONTAL, variable=hmax,
                       label='HMax', length=300, from_=0, to=255)

slider_hmax.set(255)

slider_hmin = tk.Scale(window, orient=tk.HORIZONTAL, variable=hmin,
                       label='HMin', length=300, from_=0, to=255)

''' S value sliders '''
slider_smax = tk.Scale(window, orient=tk.HORIZONTAL, variable=smax,
                       label='SMax', length=300, from_=0, to=255)

slider_smax.set(255)

slider_smin = tk.Scale(window, orient=tk.HORIZONTAL, variable=smin,
                       label='SMin', length=300, from_=0, to=255)

''' V value sliders '''
slider_vmax = tk.Scale(window, orient=tk.HORIZONTAL, variable=vmax,
                       label='VMax', length=300, from_=0, to=255)

slider_vmax.set(255)

slider_vmin = tk.Scale(window, orient=tk.HORIZONTAL, variable=vmin,
                       label='VMin', length=300, from_=0, to=255)

''' Misc '''
name_text_box = tk.Entry(window, textvariable=name)
btn_save = tk.Button(window, text='Salvar Máscara', command=save)
btn_rst = tk.Button(window, text='Resetar Máscara', command=rst)
btn_calb = tk.Button(window, text='Calibrar Origem', command=calibrate)

menu_bar = tk.Menu(window)

file_menu = tk.Menu(window, tearoff=0)
file_menu.add_command(label='Abrir', command=load_file)
file_menu.add_command(label='Salvar como', command=save_as)
menu_bar.add_cascade(label='Arquivo', menu=file_menu)

help_menu = tk.Menu(window, tearoff=0)
help_menu.add_command(label='Sobre', command=show_about)
menu_bar.add_cascade(label='Ajuda', menu=help_menu)

# Configurações de posicionamento dos widgets

label_RGB.grid(row=0, column=0)
label_Mask.grid(row=0, column=1)
canvas_rgb.grid(row=1, column=0)
canvas_hsv.grid(row=1, column=1)
slider_hmax.grid(row=2, column=1)
slider_hmin.grid(row=2, column=0)
slider_smax.grid(row=3, column=1)
slider_smin.grid(row=3, column=0)
slider_vmax.grid(row=4, column=1)
slider_vmin.grid(row=4, column=0)
name_text_box.grid(row=5, column=0)
btn_save.grid(row=6, column=0)
btn_rst.grid(row=5, column=1)
btn_calb.grid(row=7, column=0)

window.config(menu=menu_bar)  # função de configuração da barra de menus

while True:
    # Pega quadros da entrada de vídeo
    ret, frame = get_frame()

    # Redimensiona o quadro
    frame = rescale_frame(ret, frame, resize_factor)

    # Aplica a máscara em tempo real de acordo com os sliders
    res = frame_prcess(frame)

    # Converte o space color do quadro
    rtt = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Verifica se já existe alguma máscara no dicionário
    # e caso exista, carrwda as máscaras no quadro de rastreamento
    # em tempo real
    if len(masks) != 0:
        for tag, value in masks.items():
            mask = get_mask(frame, np.array(value[0:3]), np.array(value[3:6]))
            rtt = draw_contour(mask, rtt, tag)

    # Converte o quadro pra um objeto canvas

    if ret:
        photo_rgb = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(rtt))
        canvas_rgb.create_image(0, 0, image=photo_rgb, anchor=tk.NW)
        photo_hsv = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(cv2.cvtColor(res, cv2.COLOR_BGR2RGB)))
        canvas_hsv.create_image(0, 0, image=photo_hsv, anchor=tk.NW)

    window.mainloop()
