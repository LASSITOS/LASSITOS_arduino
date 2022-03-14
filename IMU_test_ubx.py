# -*- coding: utf-8 -*-
"""
Created on Thu Dec 23 12:58:45 2021
Test of ubx data parsing.

@author: Laktop

"""

import numpy as np
from datetime import datetime
import matplotlib.pyplot as pl
import os,sys
from pyubx2 import UBXReader

sys.path.append(r'C:\Users\Laktop\GNSS_arduino')
sys.path.append(r'C:\Users\AcCap\GNSS_arduino')
from UBXdata import *

# PC_loc=filepath=r'C:\Users\AcCap'
PC_loc=filepath=r'C:\Users\Laktop'
PC_loc=filepath=r'C:\Users\AcCap'
# %% read data
filepath=r'C:\Users\Laktop\GNSS_arduino\data_examples\test_IMU'



DYN4=UBXdata(filepath+r'\220110_0759.ubx')
DYN0=UBXdata(filepath+r'\220110_0818.ubx')
DYN6=UBXdata(filepath+r'\220110_0819.ubx')


DYN4_b=UBXdata(filepath+r'\220110_0458.ubx')
DYN0_b=UBXdata(filepath+r'\220110_0655.ubx')
DYN6_b=UBXdata(filepath+r'\220110_0705.ubx')


# %% plot attitudes

DYN4.plot_att()
DYN0.plot_att()
DYN6.plot_att()


DYN4_b.plot_att()
DYN0_b.plot_att()
DYN6_b.plot_att()


# %% test IMU calibration
filepath=r'C:\Users\AcCap\GNSS_arduino\data_examples\test_IMU'

# no calibration. At home before starting the car
nocal0=UBXdata(filepath+r'\220218_1949.ubx')
nocal=UBXdata(filepath+r'\220218_2034.ubx')

# Car ride to GI
drive=UBXdata(filepath+r'\220218_2054.ubx')

# Car to office walking inside
walk=UBXdata(filepath+r'\220218_2108.ubx')


# %% test PVAT
filepath=r'C:\Users\AcCap\GNSS_arduino\data_examples\test_IMU'

# no calibration. In office without GNSS reception
PVAT=UBXdata(filepath+r'\000101_1026.ubx')
check_data(PVAT)    


# %% test IMU calibration 02.03.2022
#------------------------------------
filepath=PC_loc+r'\GNSS_arduino\data_examples\test_IMU'

""""
Short tube with PVAT and OpenLog
1. Drive from GI to Fred Meyer on mode4 , system calibrating
2. Drive from Pum Station to Home Run Mode0, system calibrating
3. Hanging tube in front of house. Oschillations in different directions
4.  Hanging tube in front of house. Still until battery run low
""" 

# GI-Freds
M4_drive=UBXdata(filepath+r'\220303_0530.ubx')

# Car ride Fred-Home Run
M0_drive=UBXdata(filepath+r'\220303_0652.ubx')

# Oschillating outside
M0_osci=UBXdata(filepath+r'\220303_0720.ubx')

# still all night
M0_still=UBXdata(filepath+r'\220303_0727.ubx')

# angle chek in office
angecheck=UBXdata(filepath+r'\220303_1939.ubx')  # not working!!!!!!!!!!!!!!

# %% analazie data M4 drive
check_data(M4_drive)

M4_drive.plot_mapOSM(z='height',MSG='PVAT')
M4_drive.plot_mapOSM(z='iTOW',MSG='PVAT')

M4_drive.plot_elevation_time(MSG='PVAT')


# %% analazie data M0 drive
check_data(M0_drive)

M0_drive.plot_mapOSM(z='height',MSG='PVAT')
M0_drive.plot_mapOSM(z='iTOW',MSG='PVAT')

M0_drive.plot_elevation_time(MSG='PVAT')

"""\
    Tube rolled at entrance of sheep creeck (derapata in curva)
    Afterwards GPS drifted strongly. (Probably no signal and IMU giving bad correction data. I wonder if it would happen in mode 4.) 
    """
    
 # %% analazie data M0_osci
check_data(M0_osci)

M0_osci.plot_mapOSM(z='height',MSG='PVAT')
M0_osci.plot_mapOSM(z='iTOW',MSG='PVAT')

M0_osci.plot_elevation_time(MSG='PVAT')


 # %% analazie data M0_still
check_data(M0_still)

M0_still.plot_mapOSM(z='height',MSG='PVAT')
M0_still.plot_mapOSM(z='iTOW',MSG='PVAT')

M0_still.plot_elevation_time(MSG='PVAT')

 # %% checking roll and pitch angle in office
check_data(angecheck)

# not working!!!!!!!!!!!!!!
