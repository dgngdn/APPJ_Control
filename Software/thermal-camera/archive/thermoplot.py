#!/usr/bin/env python3

from __future__ import division
import os
import re
import math
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import argparse

parser = argparse.ArgumentParser(description='Process Flir Lepton thermal images.')
parser.add_argument('--leptonversion', type=float, default=2.5,
                        help='v2.5 and above are radiometry-capable (default: 2.5)')
parser.add_argument('--dir', type=str, default=os.getcwd(),
                        help='directory containing experimental data (default: cwd)')
parser.add_argument('--dynamic', action='store_true',
                        help='dynamically scale the colormap (default: false)')
parser.add_argument('--contour', action='store_true',
                        help='draw contours on the plot colormap (default: false)')
parser.add_argument('--singlefile', action='store_true',
                        help='pull multiple thermographs from a single file (default: false)')
parser.add_argument('--timecourse', action='store_true',
                        help='plot the extracted thermal timecourse (default: false)')
parser.add_argument('--noimages', action='store_true',
                        help='skip (default: false)')
parser.add_argument('--discrete', type=int, default=0,
                        help='discretize the colormap (default: continuous)')
parser.add_argument('--cmap', type=str, default='plasma',
                        help='set the colormap (default: plasma)')
parser.add_argument('--tmax', type=float, default=35,
                        help='maximum temperature for colormap scaling (default: 35)')
parser.add_argument('--tmin', type=float, default=0,
                        help='minimum temperature for colormap scaling (default: 0)')


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
    files = sorted(os.listdir(args.dir))
    datafiles = []
    for f in files:
        if ((len(re.findall('.bin', f)) != 0) & (len(re.findall('.bin.', f)) == 0)):
            datafiles.append(os.path.join(args.dir,f))
    return datafiles

def get_data(filename):
    data = np.genfromtxt(filename,delimiter=",",dtype=int)
    #print(data)
    f = np.vectorize(get_temp)
    return f(data)

def find_thermograph_temp(data):
    avgtemp = np.mean(data[28:33, 38:43])
    return avgtemp

def plot_timecourse(data):
    fig1 = plt.figure()
    plt1 = plt.subplot(1,1,1)
    plt1.plot(data[:,0],data[:,1],label="thermograph")
    thermopile_data = np.genfromtxt(os.path.join(args.dir,'thermopile'),delimiter=',')
    plt1.plot(thermopile_data[:,0],thermopile_data[:,1],label="ambient")
    plt1.plot(thermopile_data[:,0],thermopile_data[:,2],label="thermopile")
    plt1.legend()
    plt1.set_title("temperature comparison")
    fig1.savefig(os.path.join(args.dir,'timecourse.png'))
    plt.close()

def plot_thermal(data,filename):
    # prepare colormap
    if (args.discrete != 0):
        cm = cmap_discretize(args.cmap,args.discrete)
    else:
        cm = args.cmap

    # plot my colormapped data
    fig1 = plt.figure()
    plt1 = plt.subplot(1,1,1)
    img = plt1.imshow(data,interpolation='none',cmap=cm)

    # make a color bar
    cb = plt.colorbar(img,cmap=cm)
    if args.dynamic:
        cb_min,cb_max = cb.get_clim()
        img.set_clim(vmin = math.floor(cb_min), vmax = math.ceil(cb_max))
    else:
        img.set_clim(vmin=args.tmin,vmax=args.tmax)
    cb.set_label("temperature, degC")
    cb.draw_all()
    fig1.canvas.draw()
    #print("colorbar: {}".format(cb.get_clim()))

    # add contours
    if (args.contour):
        plt.contour(data,cmap=cm) # levels, colors, origin, extent

    # save the current figure
    fig1.savefig(''.join(str(filename).split(".")[:-1])+ ".png")
    plt.close()

def cmap_discretize(cmap, N):
    """Return a discrete colormap from the continuous colormap cmap.
        cmap: colormap instance, eg. cm.jet.
        N: number of colors.

    Example
        x = resize(arange(100), (5,100))
        djet = cmap_discretize(cm.jet, 5)
        imshow(x, cmap=djet)
    """

    if type(cmap) == str:
        cmap = plt.get_cmap(cmap)
    colors_i = np.concatenate((np.linspace(0, 1., N), (0.,0.,0.,0.)))
    colors_rgba = cmap(colors_i)
    indices = np.linspace(0, 1., N+1)
    cdict = {}
    for ki,key in enumerate(('red','green','blue')):
        cdict[key] = [ (indices[i], colors_rgba[i-1,ki], colors_rgba[i,ki]) for i in xrange(N+1) ]
    # Return colormap object.
    return matplotlib.colors.LinearSegmentedColormap(cmap.name + "_%d"%N, cdict, 1024)


#####################################
### HANDLING MONOLITHIC DATAFILES ###
#####################################

def extract_singlefile(fname):
    """Returns a list of timestamps and thermograph arrays from monolithic thermograph timecourse"""
    l = []
    fx = np.vectorize(get_temp)
    with open(fname,'r') as f:
        for line in f:
            data = line.split(',')
            ts = data[0]
            a = np.array(data[1:-1], dtype=int)
            try:
                l.append([ts, fx(a).reshape(60,80)])
            except:
                pass
    return l

def get_singlefile():
    """Returns filepath of a monolithic thermograph timecourse"""
    datafile = os.path.join(args.dir,'thermograph')
    return datafile

if __name__ == '__main__':
    if (args.singlefile):
        datafile = get_singlefile()
        thermograph_timecourse = extract_singlefile(datafile)
        if (not args.noimages):
            for elem in thermograph_timecourse:
                ts = elem[0]
                data = elem[1]
                fname = "{}.png".format(os.path.join(args.dir,ts))
                print("printing {}...".format(fname))
                plot_thermal(data,fname)
        if (args.timecourse):
            thermograph_point_timecourse = []
            for elem in thermograph_timecourse:
                ts = elem[0]
                data = elem[1]
                thermograph_point_timecourse.append([ts,find_thermograph_temp(data)])
                print("appending thermograph from {} to timecourse...".format(ts))
            plot_timecourse(np.array(thermograph_point_timecourse))
    else:
        datafiles = get_files()
        for f in datafiles:
            data = get_data(f)
            #print(data)
            fname = ''.join(str(f).split(".")[:-1])+ ".png"
            print("printing {}...".format(fname))
            plot_thermal(data,fname)
