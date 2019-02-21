# Código para testar a recepção de dados Ardiuno -> Python3
# -*- coding: utf-8 -*-

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
    try:
        print(ser.readline())
        time.sleep(1)
    except ser.SerialTimeoutException:
        print('Data could not be read')
        time.sleep(1)
