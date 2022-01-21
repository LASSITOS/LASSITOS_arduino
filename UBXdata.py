# -*- coding: utf-8 -*-
"""
Created on Tue Jan 11 12:09:02 2022

Class and function used for handling UBX data from ublox GNSS modules. 

@author: Laktop
"""

import numpy as np
from datetime import datetime
import matplotlib.pyplot as pl
import os,sys
from pyubx2 import UBXReader

# from mpl_toolkits.basemap import Basemap


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
                        try:
                            getattr(self,attr)[i]=getattr(p, attr)
                        
                        except Exception as e: 
                            print(e)
                            print('Failed to parse')
                            print(attr)
                

class UBXdata:
    
    MSG_list=['PVT','ATT','MEAS','INS']
    MSG_id_list=['NAV-PVT','NAV-ATT','ESF-MEAS','ESF-INS']
    extr_list=['ATT','PVT','INS']
    
    def __init__(self,filepath,name=''):
        """
        
        """
        if name!='': 
            self.name=name
        else:
             self.name=filepath.split('\\')[-1]   
        
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



# %% #########function definitions #############

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
    pl.title(data.name)
    
    
    
    
    
    
    
    
def check_data(data):
    """
    Check data and produce some plots
    
    Input:
    --------------
    data        Member of class UBXdata
    
    """
    
    
    print('\n\n###############################\n-----------------------------',
          data.name,
          '\n----------------------------\n###############################\n')
    for attr in ['PVT','ATT','MEAS','INS']:
        try:   
            d=getattr(data,attr)
            print('\n',attr,'\n----------------------------')
            print('Length:')
            print(len(d.parsed))
            print('Time intervall (s):')
            print((d.iTOW[:5]-d.iTOW[0])/1000)
        except Exception as e: 
                print(e)
            
            
    print('\nOthers: \n----------------------------')   
    for c in data.other[:10]: 
        print(c.identity,', bity length:',c.length) 
    
    print('\nCorrupted: \n----------------------------')   
    for c in data.corrupt[:10]: 
        print(c)      
        
    data.plot_att()
