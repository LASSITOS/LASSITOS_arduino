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




# %% analazie data logGNSS_v3, RXM data


filepath=r'C:\Users\AcCap\Google Drive\UAV\mockBird\data'
data=UBXdata(filepath+r'\220128_0016.ubx')

check_data(data)


data2=UBXdata(filepath+r'\220127_2321.ubx')
check_data(data2)
