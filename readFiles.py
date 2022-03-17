# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 11:07:51 2022

@author: AcCap
"""

import serial
import time
import os

def read_file(file,path='',printcontent=False,com='COM8',baud=115200):
    
    # setting up serial port
    ser = serial.Serial(com, baud, timeout=0) 
    
    try:
        print('Reading file: '+file)
       
        r=False
        sweep=True
        
        if path !='':
            if not os.path.isdir(path):
                os.makedirs(path)
        
        f = open(path+r'/'+file,'wb')
        
        
        bitstring=b'READ:/'+file.encode()+b':'
        ser.write(bitstring) 
        # print(bitstring)
        
        bites_written=0
        while sweep: 
            l=ser.readline()
            if l!=b'':
                if printcontent:
                    print(l)
                
                if l[:10] == b'## Reading' :
                    r=True
                    print('start reading')
                elif l[:6] == b'## End':
                    sweep=False
                    print('end of file')
                elif r:
                    # print('print to file')
                    bites_written+= f.write(l)
            
            time.sleep(0.01) 
    
        f.close()
        ser.close()
        print('Bites written: {:d}'.format(bites_written)  )
        
    except Exception as e: 
        print('error')
        print(e)
        ser.close()
