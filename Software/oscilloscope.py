#/usr/bin/env python

"""
Oscilloscope Interface

This script grabs data from the Rigol oscilloscope and saves to a file
"""

import numpy as np
from matplotlib import pyplot as plt
import datetime
import time
import os
import argparse
import usbtmc
import visa

TIMEOUT = 1.00 #0.35 without --align!
DATALENGTH = 16000
DATALENGTHPREAMBLE = 20
READWAIT = 0.15 # wait time between oscilloscope read and write, seconds
XUNIT = 's' # x-axis label
YUNIT = {1:'potential,volts',2:'potential,volts',3:'potential,volts',4:'potential,volts'} # y-units for each channel

def get_opts():
    parser = argparse.ArgumentParser(description="collects and logs data from the Rigol oscilloscope",
			    epilog="Example: python oscilloscope.py --chan 1 4 --loop")
    parser.add_argument("--chan", nargs='+', type=int, help="oscillocope channels to acquire",
                            action="store",dest="channels",required=True)
    parser.add_argument("--platform", default="usbtmc",
                            help="platform for connecting to oscilloscope")
    parser.add_argument("--plot", help="plot the acquired waveforms as they are collected",
                            action="store_true")
    parser.add_argument("--loop", help="continuously log data",
                            action="store_true")
    parser.add_argument("--align", help="guarantee all channels at same timepoint",
                            action="store_true")
    parser.add_argument("--timeout", type=float, help="timout (seconds) on oscilloscope operations",
                            default=0.4)
    parser.add_argument("--dir", type=str, default="data",
                            help="<optional> relative path to save the data")
    opts = parser.parse_args()
    print_opts(opts)
    return opts

def print_opts(opts):
    print("interfacing with: {}".format(opts.platform))
    print("acquiring channels: {}".format(opts.channels))
    if opts.plot:
        print("plotting waveforms")
    if opts.loop:
        print("continuously acquiring data")

def savedir_setup(directory):
    # path to the directory to save files
    savedir = os.path.join(os.getcwd(),directory,"oscilloscope")
    if not os.path.exists(savedir):
        print("Creating directory: {}".format(savedir))
        os.makedirs(savedir)
    return savedir

def get_oscilloscope(opts):
    if opts.platform == 'visa':
        # create device manager object
        try:
            rm = visa.ResourceManager()
        except:
            rm = visa.ResourceManager('@py')
        # create instrument object
        # rm.list_resources()
        # chamber jet
        instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA164457681::INSTR')
        # control jet
        #instr = rm.open_resource('USB0::0x1AB1::0x04CE::DS1ZA170603287::INSTR',
        #                          timeout=2000, chunk_size=102400)
        print("device info: {}".format(instr.query("*IDN?")))
        print("device timeout: {}".format(instr.timeout))
        print("device chunk size: {}".format(instr.chunk_size))
    else:
        instr = usbtmc.Instrument(0x1ab1, 0x04ce)
        instr.open()
        while not (instr.timeout == opts.timeout and instr.rigol_quirk == False):
            instr.timeout = opts.timeout
            instr.rigol_quirk = False
        id = ''
        while not id:
            try:
                id = instr.ask("*IDN?")
            except Exception as e: # USBError
                print("{} in get_oscilloscope".format(e))
                time.sleep(opts.timeout)
        print("device info: {}".format(id))
        print("device timeout: {}".format(instr.timeout))
    return instr

def prep_read(instr,platform,channel):
    """prepares the oscilloscope from reading a channel's data"""
    instr.write(":WAV:SOUR CHAN{}".format(channel))

def read_from_channel(instr,platform,preamble):
    """reads from specified oscilloscope channel;
       returns numpy array containing scaled (x,y) data"""
    ydata = []
    while len(ydata) < 1200:
        if platform == 'visa':
            ydata = instr.query_ascii_values(":WAV:DATA?",separator=wave_clean,container=np.array)
        else:
            rawdata = ''
            while len(rawdata) < DATALENGTH:
                try:
                    rawdata = instr.ask(":WAV:DATA?")
                except Exception as e:
                    print("{} in read_from_channel".format(e))
                    #read_from_channel(instr,platform,channel,preamble)
                    #capture_oscilloscope()
                    #instr.write(":RUN")
                    #time.sleep(0.1)
                    #instr.write(":STOP")
                    #break
                    return ''
            ydata = np.fromstring(rawdata[11:],dtype=float,sep=',')
    xdata = generate_xdata(len(ydata),preamble)
    yscaled = ydata #wavscale(measured=ydata,pre=preamble)
    data = np.array(list(zip(xdata,yscaled)), dtype=[('x',float),('y',float)])
    return data

def plot_data(data,ylabel,fname,savedir):
    print("plotting {}.png".format(fname))
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    ax.plot(data['x'],data['y'],linestyle='none',marker='.',markersize=5)
    ax.set_title(fname)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("time, {}".format(XUNIT))
    ax.grid(True)
    ax.get_xaxis().get_major_formatter().set_powerlimits((0, 0))
    fig.savefig(os.path.join(savedir,"{}.png".format(fname)),dpi=300)
    plt.show()
    plt.close(fig)

def save_data(data,fname,savedir):
    print("saving {}.csv".format(os.path.join(savedir,fname)))
    np.savetxt(os.path.join(savedir,"{}.csv".format(fname)),data,delimiter=',',fmt='%.5e')

def generate_xdata(points,pre):
    xincr = pre[4]
    xorig = pre[5]
    xref = pre[6]
    x = np.linspace(xorig,xincr*points,points)
    return x

#def scale_waveform(measured,pre):
#    yincr = pre[7]
#    yorig = pre[8]
#    yref = pre[9]
#    scaled = (measured - yorig - yref) * yincr
#    return measured

def preamble_clean(s):
    return filter(None, s.split('\n')[0].split(','))

def wave_clean(s):
    return filter(None, s[11:].split('\n')[0].split(','))

def instr_query(instrument, opts, msg):
    time.sleep(READWAIT)
    reply = ''
    while not reply:
        if opts.platform == "visa":
            reply = instrument.query(msg)
        else:
            try:
                reply = instrument.ask(msg)
            except Exception as e:
                print("{} in instr_query".format(e))
                time.sleep(opts.timeout)
    return reply

def instr_run(instrument, opts):
    setrun = False
    while not setrun:
        instrument.write(":RUN")
        status = instr_query(instrument, opts, ":TRIG:STAT?").strip()
        setrun = (status != "STOP")

def get_preambles(instrument, opts):
    """for each channel, grab the preamble containing the oscilloscope scaling information
    save to a dictionary for waveform scaling!"""
    preambles = {}
    for channel in opts.channels:
        instrument.write(":WAV:SOUR CHAN{}".format(channel))
        if (opts.platform == 'visa'):
            preamble = instrument.query_ascii_values(":WAV:PRE?",separator=preamble_clean)
        else:
            rawdata = ''
            while len(rawdata) < DATALENGTHPREAMBLE:
                #instr.write(":WAV:PRE?")
                try:
                    rawdata = instrument.ask(":WAV:PRE?")
                except:
                    print("{} in get_preambles".format(e))
                    time.sleep(READWAIT)
            preamble = np.fromstring(rawdata,dtype=float,sep=',')
        preambles[str(channel)] = preamble
    return preambles

def get_waveforms(instrument, opts, preambles):
        savedir = savedir_setup(opts.dir)
        instrument.write(":STOP")
        curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
        # read each channel and then save, plot (if desired)
        for channel in opts.channels:
            data = ''
#            if channel == 1:
#                instrument.timeout = opts.timeout * 1.5
#            else:
#                instrument.timeout = opts.timeout
            prep_read(instrument,opts.platform,channel)
            while len(data) == 0:
                data = read_from_channel(instrument,opts.platform,preambles[str(channel)])
                # if read_from_channel returns nothing, something went wrong
                if len(data) == 0:
                    if opts.align:
                        print("ERROR, trashing reads...")
                        #instr_reset(instrument)
                        return
                    else:
                        # reset instrument
                        instr_reset(instrument)
                        # reset curtime, and then try again!
                        curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
            fname = "{}_chan{}".format(curtime,channel)
            if opts.plot:
                ylabel = YUNIT[channel]
                plot_data(data,ylabel,fname,savedir)
            save_data(data,fname,savedir)
        print("DONE.")

def instr_reset(instrument):
    instrument.write(":RUN")
    time.sleep(READWAIT)
    instrument.write(":STOP")

def capture_oscilloscope():
    opts = get_opts()
    instr = get_oscilloscope(opts)

    #wavscale = np.vectorize(scale_waveform, excluded=['pre'])
    instr.write(":WAV:MODE NORMAL")
    instr.write(":WAV:FORMAT ASCII")
    instr_run(instr, opts)
    run = True

    # we only get the preambles once, so don't mess with oscilloscope controls!
    preambles = get_preambles(instr, opts)

    # loop that grabs the waveform data
    while run:
        get_waveforms(instr,opts,preambles)
        instr_run(instr, opts)
        run = opts.loop

if __name__ == "__main__":
    capture_oscilloscope()

