# -*- coding: utf-8 -*-
"""
Created on Tue Mar 15 11:07:51 2022

@author: AcCap
"""

import serial
import time
import os

def read_file(file,path='',printcontent=False,com='COM8',baud=115200):
    # """
    # Parameters
    # ----------
    # file : TYPE
    #     DESCRIPTION.
    # path : TYPE, optional
    #     DESCRIPTION. The default is ''.
    # printcontent : TYPE, optional
    #     DESCRIPTION. The default is False.
    # com : TYPE, optional
    #     DESCRIPTION. The default is 'COM8'.
    # baud : TYPE, optional
    #     DESCRIPTION. The default is 115200.
    # Returns
    # -------
    # None.
    # Example:  read_file('a000101_0457.txt',path=r'C:\Users\AcCap\GNSS_arduino\data_examples\GNSS_laser', printcontent=False,com='COM8',baud=115200)

    # """
    
    # setting up serial port
    ser = serial.Serial(com, baud, timeout=0) 
    
    try:
        print('Reading file: '+file)
       
        r=False
        sweep=True
        
        try:
            if path !='':
                if not os.path.isdir(path):
                    os.makedirs(path)
            
            f = open(path+r'/'+file,'wb')
        except Exception as e: 
            print('File can not be created. Choose an other path. ')
            print(e)
            ser.close()
            return
        
        bitstring=b'READ:/'+file.encode()+b':'
        ser.write(bitstring) 
        print(bitstring)
        
        bites_written=0
        printbites=5000
        while sweep: 
            l=ser.readline()
            if l!=b'':
                if printcontent:
                    print(l)
                
                if l[:4] == b'#LEM' :
                    r=True
                    print('start reading')
                elif l[:6] == b'#STOP':
                    sweep=False
                    print('end of file')
                elif r:
                    # print('print to file')
                    bites_written+= f.write(l)
                
                if bites_written>printbites:
                    print('Bites written: {:d}'.format(bites_written)  ) 
                    printbites+=5000
                    
            time.sleep(0.001) 
    
        f.close()
        ser.close()
        print('Bites written: {:d}'.format(bites_written)  )
        
    except Exception as e: 
        print('error')
        print(e)
        ser.close()



