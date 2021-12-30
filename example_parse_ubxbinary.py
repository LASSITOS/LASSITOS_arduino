# -*- coding: utf-8 -*-
"""
Created on Thu Dec 23 12:58:45 2021
Test of ubx data parsing.

@author: Laktop

"""

import numpy
from datetime import datetime
import os,sys
from pyubx2 import UBXReader


filepath=r'C:\Users\Laktop\GNSS_arduino\data_examples'
filename=r'\211224_0650.ubx'
filename=r'\000101_1601.ubx'
filename=r'\211229_2210.ubx'

stream = open(filepath+filename, 'rb')

# for s in stream:
#     print(s)
elev=[]
time=[]
ubr = UBXReader(stream, ubxonly=False)
for (raw_data, parsed_data) in ubr: 
    # print(raw_data)
    try:
        print(parsed_data)
    except UBXParseError:
        print('corrupted data')
    elev.append(parsed_data.height)
    time.append(parsed_data.second)
    
print(len(elev))
print(elev)
print(time)