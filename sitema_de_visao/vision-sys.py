# -*- coding: utf-8 -*-

'''
Sistema de visão computacional para caracterização e controle de braços
robóticos baseados na plataforma Arduino.

Desenvolvido por membros e voluntários da RAS UFCG

Colaboradores:
    - Lyang Leme de Medeiros
    -

Descrição do branch:
    - Alterar os código para que possa ser usado em ambiente ROS

'''

__version__ = '0.8.1'
__author__ = '''Lyang Leme de Medeiros'''


from tkinter import *
from tkinter import messagebox as msgbx
from tkinter import filedialog
import cv2
import PIL.Image
import PIL.ImageTk
import numpy as np
import os


class App:
    ''' Define, configura e gerencia os widgets '''

    def __init__(self, window, window_title, video_source=0):
        ''' Função construtora da calsse App '''

        # Configurações da janela
        self.window = window
        self.window.title(window_title)
        self.video_source = video_source
        self.window.geometry('750x600+600+200')

        # Variáveis auxiliares

        self.resize_factor = 0.55
        self.delay = 15
        self.hmax = IntVar()
        self.hmin = IntVar()
        self.smax = IntVar()
        self.smin = IntVar()
        self.vmax = IntVar()
        self.vmin = IntVar()
        self.name = StringVar()
        self.masks = {}
        self.new_origin = (0, 0)
        self.origin = (0, 0)

        # Objetos auxiliares

        self.vid = MyVideoCapture(self.video_source)

        # Declarações e configurações de widgets

        ''' Labels '''
        self.label_RGB = Label(self.window, text='Rasteramento em tempo real')
        self.label_Mask = Label(self.window, text='Visualizador de Máscaras')

        ''' Conteiners '''
        self.canvas_rgb = Canvas(self.window, width=self.vid.width * (self.resize_factor),
                                 height=self.vid.height * (self.resize_factor))

        self.canvas_hsv = Canvas(self.window, width=self.vid.width * (self.resize_factor),
                                 height=self.vid.height * (self.resize_factor))

        ''' H value sliders '''
        self.slider_hmax = Scale(self.window, orient=HORIZONTAL, variable=self.hmax,
                                 label='HMax', length=300, from_=0, to=255)

        self.slider_hmax.set(255)

        self.slider_hmin = Scale(self.window, orient=HORIZONTAL, variable=self.hmin,
                                 label='HMin', length=300, from_=0, to=255)

        ''' S value sliders '''
        self.slider_smax = Scale(self.window, orient=HORIZONTAL, variable=self.smax,
                                 label='SMax', length=300, from_=0, to=255)

        self.slider_smax.set(255)

        self.slider_smin = Scale(self.window, orient=HORIZONTAL, variable=self.smin,
                                 label='SMin', length=300, from_=0, to=255)

        ''' V value sliders '''
        self.slider_vmax = Scale(self.window, orient=HORIZONTAL, variable=self.vmax,
                                 label='VMax', length=300, from_=0, to=255)

        self.slider_vmax.set(255)

        self.slider_vmin = Scale(self.window, orient=HORIZONTAL, variable=self.vmin,
                                 label='VMin', length=300, from_=0, to=255)

        ''' Misc '''
        self.name_text_box = Entry(window, textvariable=self.name)
        self.btn_save = Button(window, text='Salvar Máscara', command=self.save)
        self.btn_rst = Button(window, text='Resetar Máscara', command=self.rst)
        self.btn_calb = Button(window, text='Calibrar Origem', command=self.calibrate)

        self.menu_bar = Menu(window)

        self.file_menu = Menu(window, tearoff=0)
        self.file_menu.add_command(label='Abrir', command=self.load_file)
        self.file_menu.add_command(label='Salvar como', command=self.save_as)
        self.menu_bar.add_cascade(label='Arquivo', menu=self.file_menu)

        self.help_menu = Menu(window, tearoff=0)
        self.help_menu.add_command(label='Sobre', command=self.show_about)
        self.menu_bar.add_cascade(label='Ajuda', menu=self.help_menu)

        # Configurações de posicionamento dos widgets

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
        self.btn_rst.grid(row=5, column=1)
        self.btn_calb.grid(row=7, column=0)

        # Chamada de métodos

        self.update()  # função loop para atualização das imagens
        self.window.config(menu=self.menu_bar)  # função de configuração da barra de menus
        self.window.mainloop()  # função loop principal da janela

    # Métodos da classe

    def load_file(self):
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
            self.masks[tag] = new_values

    def save_as(self):
        ''' Salva as mascaras da seção atual em um arquivo .txt '''

        path = filedialog.asksaveasfilename(
            initialdir=os.getcwd(),
            filetypes=(("Text", "*.txt"), ("All Files", "*.*")),
            defaultextension=".txt",
        )

        file = open(path, 'w+')
        for tag, values in self.masks.items():
            file.write(tag + ': ' + str(values) + '\n')
        file.close()

    def show_about(self):
        ''' Mostra informações sobre o aplicativo '''

        msgbx.showinfo(
            title='Sobre',
            message='''Software de visão computacional desenvolvido por ''' +
            '''membros e voluntários do Capítulo Estudantil IEEE RAS UFCG.''')

    def save(self):
        ''' Salva a os valores da máscara no dicionário da seção '''

        mask_value = [
            self.hmax.get(), self.smax.get(), self.vmax.get(),
            self.hmin.get(), self.smin.get(), self.vmin.get(),
        ]

        self.masks[self.name.get()] = mask_value
        self.name.set('')
        self.rst()

    def rst(self):
        ''' Reseta valores das máscaras '''

        self.slider_hmax.set(255)
        self.slider_smax.set(255)
        self.slider_vmax.set(255)
        self.slider_hmin.set(0)
        self.slider_smin.set(0)
        self.slider_vmin.set(0)

    def calibrate(self):
        ''' Coloca o centro do objeto selecionado como origem das coordenadas cartesianas '''
        self.origin = self.new_origin
        pass

    def update(self):
        ''' Pega quadros da entrada de vídeo e atualiza o App '''

        # Pega quadros da entrada de vídeo
        ret, frame = self.vid.get_frame()

        # Redimensiona o quadro
        frame = self.vid.rescale_frame(ret, frame, self.resize_factor)

        # Aplica a máscara em tempo real de acordo com os sliders
        res = self.frame_prcess(frame)

        # Converte o space color do quadro
        rtt = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Verifica se já existe alguma máscara no dicionário
        # e caso exista, carrwda as máscaras no quadro de rastreamento
        # em tempo real
        if len(self.masks) != 0:
            for tag, value in self.masks.items():
                mask = self.get_mask(frame, np.array(value[0:3]), np.array(value[3:6]))
                rtt = self.draw_contour(mask, rtt, tag)

        # Converte o quadro pra um objeto canvas
        if ret:
            self.photo_rgb = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(rtt))
            self.canvas_rgb.create_image(0, 0, image=self.photo_rgb, anchor=NW)
            self.photo_hsv = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(cv2.cvtColor(res, cv2.COLOR_BGR2RGB)))
            self.canvas_hsv.create_image(0, 0, image=self.photo_hsv, anchor=NW)

        # Loop
        self.window.after(self.delay, self.update)

    def draw_contour(self, mask, res, name=''):
        ''' Encontra e desenha contorno nas áreas da resultantes da máscara '''

        # Procura os contornos
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        # Desenha os contornos
        for contour in contours:
            area = cv2.contourArea(contour)
            M = cv2.moments(contour)
            if (M['m00'] != 0):
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.new_origin = (cx, cy)
                centroid_str = '(' + str(self.origin[0] - cx) + ',' + str(self.origin[1] - cy) + ')'

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
                cv2.line(res, self.origin, (cx, cy), (255, 0, 0), thickness=2, lineType=8, shift=0)
        return res

    def get_mask(self, frame, upper, lower):
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

    def frame_prcess(self, frame):
        ''' Aplica a máscara de acordo com os valores dos sliders '''

        # Obtem os valores dos sliders de máximo
        upper = np.array([self.hmax.get(), self.smax.get(), self.vmax.get()])

        # Obtem os valores dos sliders de mínimo
        lower = np.array([self.hmin.get(), self.smin.get(), self.vmin.get()])

        # Aplica o o recorte ao quadro segundo os valores de máximo e mínimo
        mask = self.get_mask(frame, upper, lower)

        # Recorta os pixels com valores dentro do range da máscara fazendo um and bit a bit com a mask
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Desenha o contorno ao recorte
        res = self.draw_contour(mask, res)

        return res


class MyVideoCapture:
    ''' Gerencia a captura de quadros '''

    def __init__(self, video_source=0):
        """ Médoto construtor da classe """

        self.vid = cv2.VideoCapture(video_source)
        if not self.vid.isOpened():
            raise ValueError("Unable to open video source", video_source)

        # Obtem a altura e a largura do quadro capturado
        self.width = self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.height = self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)

    def rescale_frame(self, ret, frame, percent=75):
        ''' Redimenciona o quadro recebido '''
        if ret:
            width = int(frame.shape[1] * percent)
            height = int(frame.shape[0] * percent)
            dim = (width, height)
            return cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        else:
            return None

    def get_frame(self):
        ''' Obtem um novo quadro da entrada de vídeo '''

        if self.vid.isOpened():
            ret, frame = self.vid.read()
            if ret:
                return (ret, frame)
            else:
                return (ret, None)
        else:
            return (ret, None)


App(Tk(), "SVC RAS - v" + __version__, 0)
