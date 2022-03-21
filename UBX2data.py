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
        self.len=len(self.parsed)
        # print(l)
        if self.len>0:
            for attr in self.parsed[0].__dict__.keys():
                
                if attr!='_':
                    setattr(self,attr,np.zeros(self.len,dtype=type(getattr(self.parsed[0],attr))))
        
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

                
class Laser:
    def __init__(self,path,rate=5):
        self.iTOW,self.h,self.signQ,self.T,self.iTOW2=read_Laser(path,rate=rate)
               

class UBX2data:
    
    MSG_list=['PVT','ATT','MEAS','INS','ALG', 'STATUS','PVAT']
    MSG_id_list=['NAV-PVT','NAV-ATT','ESF-MEAS','ESF-INS','ESF-ALG','ESF-STATUS','NAV-PVAT']
    extr_list=['ATT','PVT','INS','PVAT']
    
    def __init__(self,filepath,name='',Laserrate=5,clean=True,load=True):
        """
            Read GNSS and Laser data from .ubx data file. Additional methods are available for plotting and handling data.    
        
            Inputs:
            ---------------------------------------------------    
            path:           file path
            Laserrate:      data reate of Laser in Hz. If  Laserrate=0, do not load Laserdata.
            clean:          Separate GNSS and Laser data to different files. Default: clean=True
                            Used to restore GNSS data wich might be brocken by Laser data.
                            If 'force', force recleanig of data even if clean files are present.
        """
        if name!='': 
            self.name=name
        else:
             self.name=filepath.split('\\')[-1]   
        
        self.Laserrate=Laserrate
        
        
        
        if load:
            if not os.path.isfile(filepath) :
                print("File not found!!!")
                raise FileNotFoundError()
            self.file_original=filepath    
            
            if (clean=='force' or (clean==True and (not os.path.isfile(filepath[:-4]+'_GNSS.ubx') or not os.path.isfile(filepath[:-4]+'_Laser.dat') ))):
                cleanFromLaser(filepath)
                filepath=self.file_original[:-4]+'_GNSS.ubx'
                filelaser=self.file_original[:-4]+'_Laser.dat'
            if clean:
                if (not os.path.isfile(filepath[:-4]+'_GNSS.ubx') or not os.path.isfile(filepath[:-4]+'_Laser.dat') ):
                    cleanFromLaser(filepath)
                    print('Cleaned .ubs file!')
                filepath=self.file_original[:-4]+'_GNSS.ubx'
                filelaser=self.file_original[:-4]+'_Laser.dat'
            else:
                filelaser=filepath
            
            
            print('Reading file: ',filepath)
            print('----------------------------')
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
                    print(i)
                    self.corrupt.append(i)
            
            self.extract()
            
            if self.Laserrate>0:
                self.Laser=Laser(filelaser,rate=self.Laserrate)
            
                
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



    def subset(self, timelim=[],timeformat='ms'):
        for MSG in ['PVAT','PVT']: 
            if  getattr(self,  MSG).len>0:
                 d=getattr(self, MSG)
                 if timeformat=='ms':
                        lim=d.iTOW.searchsorted(timelim)
                        iTOW_lim=timelim
                 if timeformat=='s':
                        lim=((d.iTOW -d.iTOW[0])/1000).searchsorted(timelim)
                        iTOW_lim=d.iTOW[lim]
                 break
        
        data2=UBX2data(self.file_original,name=self.name,Laserrate=self.Laserrate,load=False)

        for attr in (self.MSG_list+['Laser']):
            print(attr)
            try:   
                msg_data=getattr(self,attr)
                lim2=d.iTOW.searchsorted(iTOW_lim)
            except   AttributeError:
                try:
                    msg_data=getattr(self,attr)
                    msg_data.extract()
                    lim2=d.iTOW[iTOW_lim]
                except Exception as e: 
                    print(e)
                    continue
            d2=MSG_type()
            
            for a in msg_data.__dict__.keys():
                print('/t\t',a)
                try:
                    setattr(d2,a,getattr(msg_data, a)[lim2[0]:lim2[1]] )
                except TypeError:
                    setattr(d2,a,getattr(msg_data, a))
            setattr(d2,'len',lim2[1]-lim2[0])
            setattr(data2,attr,d2)
        return data2
        
# %% #########function definitions #############

def cleanFromLaser(path):
    """
    Delete laser data from file restoring ubx data.
    
    path: file path
   
    
    """
    
    f_GNSS=open(path[:-4]+'_GNSS.ubx',mode='wb')
    f_Laser=open(path[:-4]+'_Laser.dat',mode='wb')
    i=0
    d=0
    d2=0
    end2=0
    looking=True
    with open(path,mode='rb') as f:
        s = f.read()
    while looking:
        start=s.find(b'# iTOW',end2)-3
        end=s.find(b'# end',end2)
        if start==-1 or end==-1:
            looking=False
            break
        end2=s.find(b'\r\n',end,end+30)+2
        f_Laser.write(s[start+3:end2])
        d+=end2-start-3
        if start>1:
            f_GNSS.write(s[i:start])
            d2+=start-i
        i=end2
    f_GNSS.write(s[end2:len(s)])
    d2+=len(s)-end2
    f_GNSS.close()
    f_Laser.close()
    print('Number of Laser bits:',d)
    print('Number of GNSS bits:',d2)    
    


def read_Laser(path,rate=5):
    """
    path: file path
    rate: data reate of Laser in Hz
    
    return  time of week (ms), height, signal quality, temperature 
    
    """
    
    file=open(path,mode='rt',errors='ignore')
    
    h=[]
    T=[]
    signQ=[]
    iTOW=[]
    iTOW2=[]
    i=0
    j=0
    t=0
    lon=False
    for l in file:
        if l[0]=='D' and lon:
            try:
                a=l.split()
                h.append(float(a[1]))
                signQ.append(float(a[2]))
                T.append(float(a[3])  )
                iTOW.append(0) 
                iTOW2.append(i*1000/rate)
            except:
                h.append(-999)
                signQ.append(-999)
                T.append(-999  )
                iTOW.append(0) 
                iTOW2.append(i*1000/rate)
                print('coud not parse string:', l)
            i+=1
            j+=1
        
                                
        elif l[:6]=='# iTOW':
            t=float(l.split()[2])
            # print(t)
            lon=True
        elif l[:5]=='# end':
            for k in range(1,j+1):
                 iTOW[-k]=t-(k-1)*1000/rate
            lon=False     
            j=0
        try:
            T2=np.array(iTOW2)+iTOW[0]
        except IndexError:
            T2=[]
            
    return np.array(iTOW),np.array(h),np.array(signQ),np.array(T),np.array(T2)
    
    
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
        

    try:
        if len(data.Laser.h)==0:
            raise AttributeError
                
        pl.figure()
        ax=pl.subplot(111)
        ax2=ax.twinx()
        ax.set_ylabel('Laser h (m)')
        ax2.set_ylabel('GNSS Delta_h (m)')
        ax.set_xlabel('time (ms)')
    
        ax.plot(data.Laser.iTOW-data.PVAT.iTOW[0],data.Laser.h,'--xk',label='Laser')
        # ax.plot(data.Laser.iTOW2-data.PVAT.iTOW[0],data.Laser.h,'--ob',label='Laser, time2')
        ax2.plot((data.PVAT.iTOW-data.PVAT.iTOW[0]),data.PVAT.height/1000-(data.PVAT.height[0]/1000-data.Laser.h[0]),'+:r',label='GPS height')
        
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax2.legend(lines + lines2, labels + labels2, loc=0)   
        pl.title(data.name)
    except AttributeError:
        print('No laser data found')






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
    
    
def plot_att_laser(data,MSG='PVAT',ax=[]):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
        fig=pl.gcf()
    
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
    ax2.yaxis.label.set_color('b')
    ax2.tick_params(axis='y', colors='b')

    
    
    try:
        if len(data.Laser.h)==0:
            raise AttributeError
                
        ax3=ax.twinx()
        fig.subplots_adjust(right=0.75)
        ax3.spines['right'].set_position(("axes", 1.2))
        ax3.yaxis.label.set_color('g')
        ax3.tick_params(axis='y', colors='g')
        ax3.set_ylabel('Laser h (m)')
        ax3.plot((data.Laser.iTOW-data.PVAT.iTOW[0])/1000,data.Laser.h,':og',label='Laser')
        lines3, labels3 = ax3.get_legend_handles_labels()
    except AttributeError:
        print('No laser data found')
        lines3=[]
        labels3=[]   
    # ask matplotlib for the plotted objects and their labels
    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2+ lines3, labels + labels2+ lines3, loc=0)   
    pl.title(data.name)    
    
def plot_elevation_time(data,MSG='PVT',ax=[],title=[]):
    if ax==[]:
        fig=pl.figure()
        ax=pl.subplot(111)
    else:
        pl.sca(ax)
    
    d=getattr(data,MSG)

    
    ax.plot((d.iTOW-d.iTOW[0])/1000,d.height/1000,'o-r',label='height')
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
def plot_map(data,MSG='PVT',z='height',ax=[],cmap= cm.batlow,title=[],timelim=[],timeformat='ms'):
    
    if ax==[]:
        fig=pl.figure()
        ax = pl.axes(projection=ccrs.PlateCarree())
    else:
        ax.axes(projection=ccrs.PlateCarree())
    
    
    
    d=getattr(data,MSG)   
    if not timelim==[]:
        if timeformat=='ms':
            lim=d.iTOW.searchsorted(timelim)
        if timeformat=='s':
            lim=((d.iTOW -d.iTOW[0])/1000).searchsorted(timelim)
            
            
            
    c=getattr(d,z)[lim[0]:lim[1]] # get data color plot
    
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
    im=ax.scatter(d.lon[lim[0]:lim[1]],d.lat[lim[0]:lim[1]],c=c,cmap=cmap,marker='x')
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
    
    
def plot_mapOSM(data,MSG='PVAT',z='height',ax=[],cmap= cm.batlow,title=[]):
    
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

