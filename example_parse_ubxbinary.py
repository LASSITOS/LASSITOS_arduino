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

# %%
filepath=r'C:\Users\Laktop\GNSS_arduino\data_examples'
filename=r'\211224_0650.ubx'
filename=r'\000101_1601.ubx'
filename=r'\211230_0055.ubx'
filename=r'\220102_0644.ubx'


stream = open(filepath+filename, 'rb')

# for s in stream:
#     print(s)
elev=[]
time=[]
PVT=[]
ATT=[]
pitch=[]
time2=[]

ubr = UBXReader(stream, ubxonly=False, validate=0)
for (raw_data, parsed_data) in ubr: 
    # print(raw_data)
    try:
        # print(parsed_data)
        if parsed_data.identity=='NAV-ATT':
            ATT.append(parsed_data)
            pitch.append(parsed_data.pitch)
            time2.append(parsed_data.iTOW)
        elif parsed_data.identity=='NAV-PVT':
            PVT.append(parsed_data)
            elev.append(parsed_data.height)
        time.append(parsed_data.iTOW)
    except UBXParseError:
        print('corrupted data')



# print(elev)

# print(pitch)
print(time) 
print(time2) 
print(len(time))
print(len(time2))

 print([time[i]-time2[i] for i in range(5)])
