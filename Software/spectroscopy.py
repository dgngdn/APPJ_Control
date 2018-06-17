#!/usr/bin/env python

"""
Optical Emission Spectroscopy Interface

Collects and saves data from the Ocean Optics spectrometer
"""

from __future__ import division
import subprocess
import os
import numpy as np
import matplotlib
import argparse
import datetime

parser = argparse.ArgumentParser(description="collects and logs data from Ocean Optics spectrophotometer",
			epilog="Example: python spectroscopy.py --loop")
parser.add_argument("--loop", help="continuously log data",
                        action="store_true")
parser.add_argument("--dir", type=str, default="data",
                        help="<optional> relative path to save the data")
parser.add_argument("--autoscale", action="store_true",
			help="autoscale the integration time")
parser.add_argument("--integrate", type=int, default="200000",
                        help="integration time in microseconds (default: 200,000)")
opts = parser.parse_args()

integrate_micros = opts.integrate
PEAKABSMAX = 65535
PEAKMAX = PEAKABSMAX / 4
PEAKMIN = PEAKABSMAX / 10

cwd = os.getcwd()
LIBPATH = os.path.join(cwd,"lib")
OOLIB = os.path.join(LIBPATH,"oceanoptics")



SAVEDIR = os.path.join(os.getcwd(),opts.dir,"spectroscopy") # path to the directory to save files
if not os.path.exists(SAVEDIR):
    print("Creating directory: {}".format(SAVEDIR))
    os.makedirs(SAVEDIR)

def set_integrate(itime,peak):
  if peak > PEAKMAX:
    return int(itime * min(0.5,PEAKMAX/peak))
  elif peak < PEAKMIN:
    return int(itime * 1.1)
  else:
    return int(itime)

def print_stats(integrate_micros, peak):
  print("integration time: {} us; maxpeak: {}".format(integrate_micros,peak))

def save_data(data,fname):
  print("saving {}.csv".format(os.path.join(SAVEDIR,fname)))
  np.savetxt(os.path.join(SAVEDIR,"{}.csv".format(fname)),data,delimiter=',',fmt='%.5e')

if __name__ == "__main__":
  run = True

  while run:
    p = subprocess.Popen([os.path.join(cwd,"oceanoptics-util"),"--get-spectrum","--integration-time-us",
                                          str(integrate_micros)],cwd=cwd,env={"LD_LIBRARY_PATH": OOLIB},
                                          stdout=subprocess.PIPE)
    p.wait()
    data,err = p.communicate()
    #data = np.array([e.split(',') for e in data.splitlines()],dtype=float)
    data = np.array([e.split(',') for e in data.decode('utf8').splitlines()],dtype=float)

    # save the spectrum to a csv file, timestamped with the integration time
    curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S_%f")
    fname = "{}-{}".format(curtime,integrate_micros)
    save_data(data,fname)
    peak = np.max(data[10:,1])
    print_stats(integrate_micros,peak)

    # modify integration time
    if opts.autoscale:
        integrate_micros = set_integrate(integrate_micros,peak)
    run = opts.loop

