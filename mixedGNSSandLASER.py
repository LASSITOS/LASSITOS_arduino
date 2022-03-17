# -*- coding: utf-8 -*-
"""
Created on Wed Mar  9 10:43:39 2022

@author: Ac. Capelli
"""



import numpy as np
from datetime import datetime
import matplotlib.pyplot as pl
import os,sys
from pyubx2 import UBXReader
import geopy 


sys.path.append(r'C:\Users\Laktop\GNSS_arduino')
sys.path.append(r'C:\Users\AcCap\GNSS_arduino')
from UBX2data import *




# %% check test data with mixed Laser and GNSS


filepath=r'C:\Users\AcCap\GNSS_arduino\data_examples\GNSS_laser'

#rate=1
rate1=UBX2data(filepath+r'\a000101_1259.ubx',name='rate=1Hz')
rate5=UBX2data(filepath+r'\a000101_2153.ubx',name='rate=5Hz')
rate10=UBX2data(filepath+r'\a000101_1240.ubx',name='rate=10Hz')
# rate15=UBX2data(filepath+r'\a000101_1111.ubx',name='rate=15Hz')
rate20=UBX2data(filepath+r'\a000101_0852.ubx',name='rate=20Hz')


check_data(rate1)
check_data(rate5)
check_data(rate10)
check_data(rate20)


# %% 
rate5=UBX2data(filepath+r'\a220315_0446.ubx')
check_data(rate5)