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

from mpl_toolkits.basemap import Basemap

# %% read data
filepath=r'C:\Users\Laktop\GNSS_arduino\data_examples\test_IMU'



DYN4=GNSS_data(filepath+r'\220110_0759.ubx')
DYN0=GNSS_data(filepath+r'\220110_0818.ubx')
DYN6=GNSS_data(filepath+r'\220110_0819.ubx')


DYN4_b=GNSS_data(filepath+r'\220110_0458.ubx')
DYN0_b=GNSS_data(filepath+r'\220110_0655.ubx')
DYN6_b=GNSS_data(filepath+r'\220110_0705.ubx')


# %% plot attitudes

DYN4.plot_att()
DYN0.plot_att()
DYN6.plot_att()


DYN4_b.plot_att()
DYN0_b.plot_att()
DYN6_b.plot_att()

# %%  data class

class MSG_type:
    def __init__(self):
        self.parsed=[]
        
    def extract(self):
        l=len(self.parsed)
        # print(l)
        if l>0:
            for attr in self.parsed[0].__dict__.keys():
                
                if attr!='_':
                    setattr(self,attr,np.zeros(l,dtype=type(getattr(self.parsed[0],attr))))
        
            for i,p in enumerate(self.parsed):
                for attr in p.__dict__.keys():
                    
                    if attr!='_':
                        getattr(self,attr)[i]=getattr(p, attr)
                    
    
                

class GNSS_data:
    
    MSG_list=['PVT','ATT','MEAS','INS']
    MSG_id_list=['NAV-PVT','NAV-ATT','ESF-MEAS','ESF-INS']
    extr_list=['ATT','PVT','INS']
    
    def __init__(self,filepath,name=''):
        
        self.name=name

        
        stream = open(filepath, 'rb')
        ubr = UBXReader(stream, ubxonly=False, validate=0)
        
        for msg in self.MSG_list:
            setattr(self,msg,MSG_type() )
        
        i=0
        self.corrupt=[]
        self.other=[]
        
        for (raw_data, parsed_data) in ubr: 
            # print(raw_data)
            i+=1
            # print(parsed_data)
            try:
                # print(parsed_data)
                if parsed_data.identity in self.MSG_id_list:
                    j=self.MSG_id_list.index(parsed_data.identity)
                    
                    getattr(self,self.MSG_list[j]).parsed.append(parsed_data)
                else:
                    self.other.append(parsed_data)
                        
                    
            except Exception as e: 
                print(e)
                print('Failed to parse')
                self.corrupt.append(i)
        
        self.extract()
        
                
    def extract(self):
        for msg in self.extr_list:
            try:
                getattr(self, msg).extract()
                
            except AttributeError:
                print(msg)


    def plot_att(self, MSG='ATT',ax=[]):
        plot_att(self,MSG='ATT',ax=[])



#########functions#############

def plot_att(data,MSG='ATT',ax=[]):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
    
    d=getattr(data,MSG)
    
    ax2=ax.twinx()    
    
    ax.plot(d.iTOW/1000,d.pitch,'o-r',label='pitch')
    ax.plot(d.iTOW/1000,d.roll,'x-k',label='roll')
    ax2.plot(d.iTOW/1000,d.heading,'x-b',label='heading')
    ax.set_ylabel('pitch/roll (deg)')
    ax2.set_ylabel('heading (deg)')
    ax.set_xlabel('time (s)')
   
    # ask matplotlib for the plotted objects and their labels
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc=0)   

# %% test data    
filename=r'\220110_0819.ubx'
filename=r'\220110_0818.ubx'
filename=r'\220110_0759.ubx'

DYN4=GNSS_data(filepath+r'\220110_0759.ubx')

# %% old code
  
filename=r'\220110_0819.ubx'
filename=r'\220110_0818.ubx'
filename=r'\220110_0759.ubx'
stream = open(filepath+filename, 'rb')
    
# for s in stream:
#     print(s)
PVT=[]
ATT=[]
MEAS=[]
ALG=[]
INS=[]

elev=[]
time=[]

pitch=[]
roll=[]
time2=[]
corrupt=[]
corrupted=[]
i=0
other=[]

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
ax.plot(time/1000,elev/1000,'x-b',label='elevation')
ax.set_ylabel('elevation (m)')
ax2.plot(time2/1000,pitch,'o-r',label='pitch')
ax2.plot(time2/1000,roll,'x-k',label='roll')
ax.set_ylabel('pitch (deg)')
ax.set_xlabel('time (s)')
pl.legend()

# print(pitch)
print(np.diff(time)) 
print(np.diff(time2)) 
print(len(time))
print(len(time2))

print([time[i]-time2[i] for i in range(5)])

print(corrupt)

