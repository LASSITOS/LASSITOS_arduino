# -*- coding: utf-8 -*-
"""
Created on Thu Dec 23 12:58:45 2021
Test of ubx data parsing.

@author: Laktop

"""

import numpy as np
from datetime import datetime
import matplotlib.pyplot as pl
from matplotlib.gridspec import GridSpec
import os,sys

from geopy import distance
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
from cartopy.mpl.ticker import LongitudeFormatter, LatitudeFormatter
import io
from urllib.request import urlopen, Request
from PIL import Image
from scipy import signal
from scipy.fft import fftshift
import matplotlib.colors as colors

sys.path.append(r'C:\Users\Laktop\GNSS_arduino')
sys.path.append(r'C:\Users\AcCap\GNSS_arduino')
from INSLASERdata import *



path=r'C:\Users\AcCap\GNSS_arduino'
# path=r'G:\My Drive\UAV\mockBird'

filepath=path+r'\data_examples'

# %% test data



data=INSLASERdata(filepath+r'\INS9800106_0256.dat',name='trst',correct_Laser=False,distCenter=0,roll0=0,pitch0=0)
# data=data.subset(data.PINS1.TOW[[0,-1]],timeformat='TOW')


check_data(data)


dTOW_laser=np.diff(data.Laser.TOW)
dTOW_PINS=np.diff(data.PINS1.TOW)
