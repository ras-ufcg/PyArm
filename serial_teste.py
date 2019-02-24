# Código para testar a recepção de dados Ardiuno -> Python3
# -*- coding: utf-8 -*-

# https://petrimaki.com/2013/04/28/reading-arduino-serial-ports-in-windows-7/
# https://pyserial.readthedocs.io/en/latest/shortintro.html#configuring-ports-later

import serial
import time
import os

list_port = []
ser = serial.Serial(timeout=0)
ser.baudrate = 9600
ports = os.popen('python -m serial.tools.list_ports').read()

print('Portas Com ativas:')
for i, port in enumerate(ports.split('\n')):
    if port:
        print(str(i) + '. \'' + port + '\'')
ser.port = 'COM3'

ser.open()

while 1:
    var = input("Entre com o comando para o servo: ")
    ser.write(var.encode())
