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
data=UBX2data(filepath+r'\a000101_0131.ubx',name='rate=5Hz')

# check_data(data)


