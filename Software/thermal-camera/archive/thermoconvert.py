#!/usr/bin/env python3

import os
import re
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
import argparse

parser = argparse.ArgumentParser(description='Process Flir Lepton thermal images.')
parser.add_argument('--leptonversion', type=float, default=2.5,
                        help='v2.5 and above are radiometry-capable')
parser.add_argument('--dynamicscale', type=bool, default=False,
                        help='whether to dynamically scale the colormapping')
parser.add_argument('--tmax', type=float, default=35,
                        help='maximum temperature for colormap scaling')
parser.add_argument('--tmin', type=float, default=0,
                        help='minimum temperature for colormap scaling')

args = parser.parse_args()

def get_temp(val):
    """Takes a value returned by the Flir Lepton thermal camera and returns a real temperature"""
    if (args.leptonversion >= 2.5):
      degC = val/100 - 273.15
    else:
      deviceslope = 0.022
      Tamb = 21
      degC = deviceslope*(int(val)-8192)+Tamb
    return degC

def get_files():
    """Returns a list of thermal binary files in the CWD to convert"""
    files = sorted(os.listdir(os.getcwd()))
    datafiles = []
    for f in files:
        if ((len(re.findall('.bin', f)) != 0) & (len(re.findall('.bin.', f)) == 0)):
            datafiles.append(f)
    return datafiles

def get_data(filename):
    data = np.genfromtxt(filename,delimiter=",",dtype=int)
    print(data)
    f = np.vectorize(get_temp)
    return f(data)

def plot_thermal(data,filename):
    # set some bounds for my colormapping
    numticks = 11
    # plot my colormapped data
    fig1 = plt.figure()
    plt1 = plt.subplot(1,1,1)
    
    if args.dynamicscale:
        img = plt1.imshow(data,interpolation='none',cmap=plt.cm.gnuplot)
    else:
        img = plt1.imshow(data,interpolation='none',cmap=plt.cm.gnuplot,vmin=args.tmin,vmax=args.tmax)
    
    # make a color bar
    # http://matplotlib.org/api/pyplot_api.html#matplotlib.pyplot.colorbar
    # http://pyhogs.github.io/colormap-examples.html
    # http://stackoverflow.com/questions/7875688/how-can-i-create-a-standard-colorbar-for-a-series-of-plots-in-python
    ticks = np.linspace(args.tmin,args.tmax,numticks)
    cb = plt.colorbar(img,cmap=plt.cm.gnuplot)
    if not args.dynamicscale:
        cb.set_clim(vmin=args.tmin,vmax=args.tmax)
    cb.set_ticks(ticks)
    cb.set_ticklabels(ticks)
    cb.update_ticks()

    # save the current figure
    fig1.savefig(str(filename)[:-4]+'.png')
    plt.close()

if __name__ == '__main__':
    datafiles = get_files()
    for f in datafiles:
        data = get_data(f)
        print(data)
        plot_thermal(data,f)
