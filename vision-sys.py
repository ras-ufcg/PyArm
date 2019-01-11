## vision-sys.py 
# version 0.1

import cv2
import numpy as np

# o parametro passado na funcao video capture serve para selecionar a entrada de video.
# caso esteja desenvolvendo em um pc com apenas uma webcam, deve passar o paramentro '0'
# em caso de multiplas entradas de video conectadas, o parametro deve mudar de acordo com a entrada desejada.

cap = cv2.VideoCapture(1)

# cria a janela onde para os track bars

cv2.namedWindow('filter')

# os track bar, por padrao chamam uma funcao sempre que mudam de valor
# nesse caso, nao queremos nenhuma acao especifica, entao chamaremaos a funcao abaixo 

def nothing(x):
    pass

cv2.createTrackbar('H_MAX','filter',255,255,nothing)
cv2.createTrackbar('H_MIN','filter',0,255,nothing)
cv2.createTrackbar('S_MAX','filter',255,255,nothing)
cv2.createTrackbar('S_MIN','filter',0,255,nothing)
cv2.createTrackbar('V_MAX','filter',255,255,nothing)
cv2.createTrackbar('V_MIN','filter',0,255,nothing)

while(1):

    # a funcao read() retorna dois valores, ret e frame 
    # ret eh um valor logico que eh verdade quando ocorre a captura de um frame corretamente
    # frame eh a matriz com a imagem capturada

    ret, frame = cap.read()

    # mudando o color space para trabalhar com o padrao HSV que eh melhor para detectar borda dos objetos

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # pega a posicao dos cursores nas track bars

    H = cv2.getTrackbarPos('H_MAX','filter')
    S = cv2.getTrackbarPos('S_MAX','filter')
    V = cv2.getTrackbarPos('V_MAX','filter')

    h = cv2.getTrackbarPos('H_MIN','filter') 
    s = cv2.getTrackbarPos('S_MIN','filter')
    v = cv2.getTrackbarPos('V_MIN','filter')

    # representa os valores maximos e minimo hsv em duas variaveis para gerar mascara para o video

    upper = np.array([H,S,V])
    lower = np.array([h,s,v])

    # gera a mascara 

    mask = cv2.inRange(hsv, lower, upper)

    # faz um and bit a bit entre a mascara e o video original para recortar a cor desejada

    res = cv2.bitwise_and(frame,frame, mask= mask)

    # mostra os frames em janelas separadas

    cv2.imshow('BRG Image', frame)
    cv2.imshow('HSV Image', hsv)
    cv2.imshow('Result', res)
    
    # pressione 'Esc' para sair

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()