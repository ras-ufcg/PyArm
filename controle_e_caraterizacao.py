# -*- coding: utf-8 -*-

'''
Sistema para controle de braços
robóticos baseados na plataforma Arduino.

Desenvolvido por membros e voluntários da RAS UFCG

Colaboradores:
    - Lyang Leme de Medeiros
    -

Características da versão:
    - Comunicação Serial com o Arduino

###########################################
# OBS.: Lembrar de checar a porta serial. #
###########################################

###########################################
################ PROTOCOLO ################

Os servos trabalham com uma amplitude de 180
graus.

Nesse caso estamos controlando 6 servos
Cada uma dessas posições está numerada
no shield (de 1 à 6)

Então para controlar o braço precisamos
enviar dois dados, o número do servo sobre o
qual desejamos atuar e a posição final em
graus.

Esses dados foram codificados dentro de um
intero de 4 dígitos sendo o que ocupa a posição
mais significativa resposável por informar qual
motor deverar ser acionado e as 3 posições finais
informam o angulo que o motor deverá atingir.

Por exemplo:

Enviando o comando 1090 o sevo 1 irá para sua
posição em 90°

Enviando o comando 3150 o sevo 3 irá para sua
posição em 150°

###########################################
################ LIGAÇÕES #################


                S1 - D9
                S2 - D10
                S3 - D11
                S4 - D3
                S5 - D5
                S6 - D6

'''

__version__ = '0.1'
__author__ = 'Lyang Leme de Medeiros'

from tkinter import *
import serial

_s1Valor = 90
_s2Valor = 90
_s3Valor = 90
_s4Valor = 90
_s5Valor = 90
_s6Valor = 90

ser = serial.Serial(timeout=0)
ser.baudrate = 9600
ser.port = 'COM3'
ser.open()


def testa():
    global _s1Valor, _s2Valor, _s3Valor, _s4Valor, _s5Valor, _s6Valor

    if (_s1Valor != s1Valor.get()):
        _s1Valor = s1Valor.get()
        envia(_s1Valor + 1000)

    if (_s2Valor != s2Valor.get()):
        _s2Valor = s2Valor.get()
        envia(_s2Valor + 2000)

    if (_s3Valor != s3Valor.get()):
        _s3Valor = s3Valor.get()
        envia(_s3Valor + 3000)

    if (_s4Valor != s4Valor.get()):
        _s4Valor = s4Valor.get()
        envia(_s4Valor + 4000)

    if (_s5Valor != s5Valor.get()):
        _s5Valor = s5Valor.get()
        envia(_s5Valor + 5000)

    if (_s6Valor != s6Valor.get()):
        _s6Valor = s6Valor.get()
        envia(_s6Valor + 6000)


def envia(valor):
    if(ser.is_open):
        ser.write(str(valor).encode())


app = Tk()
app.geometry('+350+200')
app.title('Controle Servos')

s1Valor = IntVar()
s2Valor = IntVar()
s3Valor = IntVar()
s4Valor = IntVar()
s5Valor = IntVar()
s6Valor = IntVar()

s1Scl = Scale(app, orient=HORIZONTAL, variable=s1Valor, label='S1', length=300, from_=0, to=180)
s2Scl = Scale(app, orient=HORIZONTAL, variable=s2Valor, label='S2', length=300, from_=0, to=180)
s3Scl = Scale(app, orient=HORIZONTAL, variable=s3Valor, label='S3', length=300, from_=0, to=180)
s4Scl = Scale(app, orient=HORIZONTAL, variable=s4Valor, label='S4', length=300, from_=0, to=180)
s5Scl = Scale(app, orient=HORIZONTAL, variable=s5Valor, label='S5', length=300, from_=0, to=180)
s6Scl = Scale(app, orient=HORIZONTAL, variable=s6Valor, label='S6', length=300, from_=0, to=180)

s1Scl.set(90)
s2Scl.set(90)
s3Scl.set(90)
s4Scl.set(90)
s5Scl.set(90)
s6Scl.set(90)

eviaBtn = Button(app, text='envia', command=testa)

m = PanedWindow(orient=VERTICAL)
m.pack(fill=BOTH, expand=1)

s1Scl.pack()
s2Scl.pack()
s3Scl.pack()
s4Scl.pack()
s5Scl.pack()
s6Scl.pack()

eviaBtn.pack()

app.mainloop()
