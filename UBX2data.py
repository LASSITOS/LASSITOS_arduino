# -*- coding: utf-8 -*-
"""
Created on Tue Jan 11 12:09:02 2022

Class and function used for handling data files with mixed UBX data from ublox GNSS modules and frm LDS70A Laser altimeter. 

@author: Laktop
"""

import numpy as np
from datetime import datetime
import matplotlib.pyplot as pl
import os,sys
from pyubx2 import UBXReader
from cmcrameri import cm
from geopy import distance
import cartopy.crs as ccrs
import cartopy.io.img_tiles as cimgt
from cartopy.mpl.ticker import LongitudeFormatter, LatitudeFormatter
import io
from urllib.request import urlopen, Request
from PIL import Image

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
                            getattr(self,attr)[i]=-999
                            print(i)
                            print(e)
                            print('Failed to parse')
                            print(attr)
                            try:
                                print(getattr(p, attr))
                            
                            except Exception as e: 
                                print(e)
                

class UBX2data:
    
    MSG_list=['PVT','ATT','MEAS','INS','ALG', 'STATUS','PVAT']
    MSG_id_list=['NAV-PVT','NAV-ATT','ESF-MEAS','ESF-INS','ESF-ALG','ESF-STATUS','NAV-PVAT']
    extr_list=['ATT','PVT','INS','PVAT']
    
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
        plot_att(self,MSG=MSG,ax=ax)
        
    def plot_elevation_time(self, MSG='PVT',ax=[],title=[]):
        plot_elevation_time(self,MSG=MSG,ax=ax,title=title)

    def plot_longlat(self,MSG='PVT',z='height',ax=[],cmap= cm.batlow):
        plot_longlat(self,MSG=MSG,z=z,ax=ax,cmap=cmap)
        
    def plot_map(self,MSG='PVT',z='height',ax=[],cmap= cm.batlow):
        plot_map(self,MSG=MSG,z=z,ax=ax,cmap=cmap)

    def plot_mapOSM(self,MSG='PVT',z='iTOW',ax=[],cmap= cm.batlow,title=[]):
        plot_mapOSM(self,MSG=MSG,z=z,ax=ax,cmap= cmap,title=title)

# %% #########function definitions #############


    
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
    for attr in data.MSG_list:
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
        print(c.identity,', bit length:',c.length) 
    
    print('\nCorrupted: \n----------------------------')   
    for c in data.corrupt[:10]: 
        print(c)      
        
    try:
        if data.PVAT.parsed != []:
            data.plot_att(MSG='PVAT')
        else:
            data.plot_att()
    except AttributeError:
        print('no attitude messages found')
        








def plot_att(data,MSG='ATT',ax=[]):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
    
    d=getattr(data,MSG)
    
    ax2=ax.twinx()    
    
    if MSG=='PVAT':
        ax.plot((d.iTOW-d.iTOW[0])/1000,d.vehPitch,'o-r',label='pitch')
        ax.plot((d.iTOW-d.iTOW[0])/1000,d.vehRoll,'x-k',label='roll')
        ax2.plot((d.iTOW-d.iTOW[0])/1000,d.vehHeading,'x-b',label='heading')
    else:
        ax.plot((d.iTOW-d.iTOW[0])/1000,d.pitch,'o-r',label='pitch')
        ax.plot((d.iTOW-d.iTOW[0])/1000,d.roll,'x-k',label='roll')
        ax2.plot((d.iTOW-d.iTOW[0])/1000,d.heading,'x-b',label='heading')
    
    
    ax.set_ylabel('pitch/roll (deg)')
    ax2.set_ylabel('heading (deg)')
    ax.set_xlabel('time (s)')
   
    # ask matplotlib for the plotted objects and their labels
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels + labels2, loc=0)   
    pl.title(data.name)
    
    
    
    
def plot_elevation_time(data,MSG='PVT',ax=[],title=[]):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
    
    d=getattr(data,MSG)

    
    ax.plot((d.iTOW-d.iTOW[0])/1000,d.height/1000,'o-r',label='pitch')
    ax.set_ylabel('elevation (m a.s.l.)')
    ax.set_xlabel('time (s)')
   
    if title:
        pl.title(title)
    pl.tight_layout()
    
    
def plot_longlat(data,MSG='PVT',z='height',ax=[],cmap= cm.batlow):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
    
    d=getattr(data,MSG)
    
    ax2=ax.twinx()    
    c=getattr(d,z)
    if z=='height':
        label='elevation (m a.s.l.)'
        c=c/1000
    elif z=='iTOW':
        label='time (s)'
        c-=c[0]
        c=c/1000
    else:
        label=z
    lat=np.array([distance.distance((d.lat[0],d.lon[0]),(x,d.lon[0])).m for x in d.lat ])
    lon=np.array([distance.distance((d.lat[0],d.lon[0]),(d.lat[0],x)).m for x in d.lon ])
    
    im=ax.scatter(lon,lat,c=c,cmap=cmap,marker='x')
    c=pl.colorbar(im, label=label)
    ax.invert_xaxis()
    ax.set_ylabel('N (m)')
    ax.set_xlabel('W (m)')
    
    pl.title(data.name)
    pl.tight_layout()

        





# %% plot on map
def plot_map(data,MSG='PVT',z='height',ax=[],cmap= cm.batlow,title=[]):
    
    if ax==[]:
        fig=pl.figure()
        ax = pl.axes(projection=ccrs.PlateCarree())
    else:
        ax.axes(projection=ccrs.PlateCarree())
        
    d=getattr(data,MSG)     
    c=getattr(d,z)
    if z=='height':
        label='elevation (m a.s.l.)'
        c=c/1000
    elif z=='iTOW':
        label='time (s)'
        c-=c[0]
        c=c/1000
    else:
        label=z
        
    ax.coastlines()
    im=ax.scatter(d.lon,d.lat,c=c,cmap=cmap,marker='x')
    c=pl.colorbar(im, label=label)
    
    lon_formatter = LongitudeFormatter(number_format='0.1f',degree_symbol='',dateline_direction_label=True) # format lons
    lat_formatter = LatitudeFormatter(number_format='0.1f',degree_symbol='') # format lats
    ax.xaxis.set_major_formatter(lon_formatter) # set lons
    ax.yaxis.set_major_formatter(lat_formatter) # set lats
    
    if title:
        pl.title(title)
    pl.tight_layout()  
    


def image_spoof(self, tile): # this function pretends not to be a Python script
        url = self._image_url(tile) # get the url of the street map API
        req = Request(url) # start request
        req.add_header('User-agent','Anaconda 3') # add user agent to request
        fh = urlopen(req) 
        im_data = io.BytesIO(fh.read()) # get image
        fh.close() # close url
        img = Image.open(im_data) # open image with PIL
        img = img.convert(self.desired_tile_form) # set image format
        return img, self.tileextent(tile), 'lower' # reformat for cartopy
    
    
def plot_mapOSM(data,MSG='PVT',z='height',ax=[],cmap= cm.batlow,title=[]):
    
    cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    osm_img = cimgt.OSM() # spoofed, downloaded street map
    
    if ax==[]:
        fig=pl.figure()
        ax = pl.axes(projection=osm_img.crs)
    else:
        ax.axes(projection=osm_img.crs)
    
    # prepare data
    d=getattr(data,MSG)     
    c=getattr(d,z)
    if z=='height':
        label='elevation (m a.s.l.)'
        c=c/1000
    elif z=='iTOW':
        label='time (s)'
        c-=c[0]
        c=c/1000
    else:
        label=z
    
    dx=(np.max(d.lon)-np.min(d.lon))
    dy=(np.max(d.lat)-np.min(d.lat))
    W=np.max([dx,dy])*3/5
    print(W)
    center=[(np.max(d.lon)+np.min(d.lon))/2, (np.max(d.lat)+np.min(d.lat))/2]
    extent = [center[0]-W,center[0]+W, center[1]-W,center[1]+W] # Contiguous US bounds
   
    
   
    # setup map
    ax.set_extent(extent) # set extents
    ax.set_xticks(np.linspace(extent[0],extent[1],3),crs=ccrs.PlateCarree()) # set longitude indicators
    ax.set_yticks(np.linspace(extent[2],extent[3],4)[1:],crs=ccrs.PlateCarree()) # set latitude indicators
    lon_formatter = LongitudeFormatter(number_format='0.4f',degree_symbol='',dateline_direction_label=True) # format lons
    lat_formatter = LatitudeFormatter(number_format='0.4f',degree_symbol='') # format lats
    ax.xaxis.set_major_formatter(lon_formatter) # set lons
    ax.yaxis.set_major_formatter(lat_formatter) # set lats
    ax.xaxis.set_tick_params()
    ax.yaxis.set_tick_params()
    
    scale = np.ceil(-np.sqrt(2)*np.log(np.divide((extent[1]-extent[0])/2.0,350.0))) # empirical solve for scale based on zoom
    scale = (scale<20) and scale or 19 # scale cannot be larger than 19
    ax.add_image(osm_img, int(scale))

    im=ax.scatter(d.lon,d.lat,c=c,cmap=cmap,marker='x',transform=ccrs.PlateCarree())
    c=pl.colorbar(im, label=label)
    
    if title:
        pl.title(title)
    pl.tight_layout()

