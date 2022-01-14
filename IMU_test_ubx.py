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

from UBXdata import *


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


