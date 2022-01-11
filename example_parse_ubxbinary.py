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

# %%
filepath=r'C:\Users\Laktop\GNSS_arduino\data_examples'
filename=r'\211224_0650.ubx'
filename=r'\000101_1601.ubx'
filename=r'\211230_0055.ubx'
filename=r'\220104_0628.ubx'
filename=r'\220104_0804.ubx'
filename=r'\220104_0829.ubx'
filename=r'\000101_0737.ubx'

stream = open(filepath+filename, 'rb')

# for s in stream:
#     print(s)
elev=[]
time=[]
PVT=[]
ATT=[]
pitch=[]
roll=[]
time2=[]
corrupt=[]
corrupted=[]
i=0
other=[]
MEAS=[]
time3=[]


ubr = UBXReader(stream, ubxonly=False, validate=0)

for (raw_data, parsed_data) in ubr: 
    # print(raw_data)
    i+=1
    # print(parsed_data)
    try:
        print(parsed_data)
        if parsed_data.identity=='NAV-ATT':
            ATT.append(parsed_data)
            pitch.append(parsed_data.pitch)
            roll.append(parsed_data.roll)
            time2.append(parsed_data.iTOW)
        elif parsed_data.identity=='NAV-PVT':
            PVT.append(parsed_data)
            elev.append(parsed_data.height)
            time.append(parsed_data.iTOW)
        elif parsed_data.identity=='ESF-MEAS':
            MEAS.append(parsed_data)
            time3.append(parsed_data.timeTag)
        else:
            other.append(parsed_data.identity)
                
            
    except:
        print('Failed to parse')
        corrupt.append(i)
        corrupted.append(parsed_data)
        
time=np.array(time)
time2=np.array(time2)
time3=np.array(time3)
pitch=np.array(pitch)
roll=np.array(roll)
elev=np.array(elev)
# print(elev)
time2-=time[0]
time3=time3
time-=time[0]


fig=pl.figure()
ax=pl.subplot(111)
ax2=ax.twinx()
ax.plot(time/1000,elev/1000,'x-b')
ax.set_ylabel('elevation (m)')
ax2.plot(time2/1000,pitch,'o-r')
ax2.plot(time2/1000,roll,'x-k')
ax.set_ylabel('pitch (deg)')
ax.set_xlabel('time (s)')

# print(pitch)
print(np.diff(time)) 
print(np.diff(time2)) 
print(len(time))
print(len(time2))

print([time[i]-time2[i] for i in range(5)])

print(corrupt)

