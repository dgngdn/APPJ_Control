#!/usr/bin/env python3.5

import sys
sys.dont_write_bytecode = True

import os
import datetime
import time
import numpy as NP
import cv2
import argparse
from pylepton import Lepton
import RPi.GPIO as gpio
gpio.setwarnings(False)
import subprocess
import select
import scipy.io as scio
from scipy import linalg
from casadi import *
# Import core code.
import core
import serial
#import asyncio
#import serial_asyncio
import crcmod
#import crcmod.predefinmaed
import usbtmc
import visa
sys.path.append('/home/brandon/repos/python-seabreeze')
import seabreeze.spectrometers as sb


U0 = NP.array([(8.0,16.0,1.2)], dtype=[('v','>f4'),('f','>f4'),('q','>f4')])

crc8 = crcmod.predefined.mkCrcFun('crc-8-maxim')

##initialize oscilloscope
instr = usbtmc.Instrument(0x1ab1, 0x04ce)
instr.open()
while not (instr.timeout == 1 and instr.rigol_quirk == False):
    instr.timeout = 1
    instr.rigol_quirk = False
idg = ''
while not idg:
    try:
        idg = instr.ask("*IDN?")
    except Exception as e: # USBError
         print("{} in get_oscilloscope".format(e))
         time.sleep(0.4)
print("device info: {}".format(idg))
print("device timeout: {}".format(instr.timeout))
time.sleep(0.5)

## initialize spectrometer
devices = sb.list_devices()
#t_int=12000
t_int=12000*4
print("Available devices {}".format(devices))
spec = sb.Spectrometer(devices[0])
print("Using {}".format(devices[0])) 
spec.integration_time_micros(t_int)
print("integratiopn time {} seconds.".format(t_int/1e6))
time.sleep(0.5)

class DummyFile(object):
    def write(self, x): pass

def nostdout(func):
    def wrapper(*args, **kwargs):
        save_stdout = sys.stdout
        sys.stdout = DummyFile()
        func(*args, **kwargs)
        sys.stdout = save_stdout
    return wrapper

def get_runopts():
  """
  Gets the arguments provided to the interpreter at runtime
  """
  parser = argparse.ArgumentParser(description="runs MPC",
			  epilog="Example: python mpc_lin_test.py --quiet")
  #parser.add_argument("--quiet", help="silence the solver", action="store_true")
  parser.add_argument("--faket", help="use fake temperature data", action="store_true")
  parser.add_argument("--fakei", help="use fake intensity data", action="store_true")
  parser.add_argument("--timeout", type=float, help="timout (seconds) on oscilloscope operations",
                            default=0.4)
  runopts = parser.parse_args()
  return runopts

def send_inputs(device,U):
  """
  Sends input values to the microcontroller to actuate them
  """
  Vn = U[0]+U0['v'][0]
  Fn = U[1]+U0['f'][0]
  Qn = U[2]+U0['q'][0]
  input_string='echo "v,{:.2f}" > /dev/arduino && echo "f,{:.2f}" > /dev/arduino && echo "q,{:.2f}" > /dev/arduino'.format(Vn, Fn, Qn)
  #subprocess.run('echo -e "v,{:.2f}\nf,{:.2f}\nq,{:.2f}" > /dev/arduino'.format(U[:,0][0]+8, U[:,1][0]+16, U[:,2][0]+1.2), shell=True)
  device.reset_input_buffer()
  #device.write("v,{:.2f}\n".format(Vn).encode('ascii'))
  subprocess.run('echo "" > /dev/arduino', shell=True)
  time.sleep(0.200)

  subprocess.run('echo "v,{:.2f}" > /dev/arduino'.format(Vn), shell=True)
  time.sleep(0.200)
  #device.write("f,{:.2f}\n".format(Fn).encode('ascii'))
  subprocess.run('echo "f,{:.2f}" > /dev/arduino'.format(Fn), shell=True)
  time.sleep(0.200)
  #device.write("q,{:.2f}\n".format(Qn).encode('ascii'))
  subprocess.run('echo "q,{:.2f}" > /dev/arduino'.format(Qn), shell=True)
  #subprocess.call(input_string,  shell=True)
  #print("input: {}".format(input_string))
  print("input values: {:.2f},{:.2f},{:.2f}".format(Vn,Fn,Qn))

def is_valid(line):
  """
  Verify that the line is complete and correct
  """
  l = line.split(',')
  crc = int(l[-1])
  data = ','.join(l[:-1])
  return crc_check(data,crc)

def crc_check(data,crc):
  crc_from_data = crc8("{}\x00".format(data).encode('ascii'))
  print("crc:{} calculated: {} data: {}".format(crc,crc_from_data,data))
  return crc == crc_from_data

def get_temp(runopts):
  """
  Gets treatment temperature with the Lepton thermal camera
  """
  if runopts.faket:
    return 24

  run = True
  while run:
    try:
      with Lepton("/dev/spidev0.1") as l:
        data,_ = l.capture(retry_limit = 3)
      if l is not None:
        Ts = NP.amax(data) / 100 - 273;
        for line in data:
          l = len(line)
          if (l != 80):
            print("error: should be 80 columns, but we got {}".format(l))
          elif Ts > 150:
            print("Measured temperature is too high: {}".format(Ts))
        #curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
        #fname = "{}".format(curtime)
        #Ts = NP.amax(data) / 100 - 273;
        #Ts = NP.true_divide(NP.amax(data[7:50]),100)-273;
        time.sleep(0.050)
        run = False
    except:
      print("\nHardware error on the thermal camera. Lepton restarting...")
      gpio.output(35, gpio.HIGH)
      time.sleep(0.5)
      gpio.output(35, gpio.LOW)
      print("Lepton restart completed!\n\n")
  return Ts

def get_intensity(f,runopts):
  """
  Gets optical intensity from the microcontroller
  """
  if runopts.fakei:
    Is = 5
  else:
    run = True
    while run:

      try:
        f.reset_input_buffer()
        f.readline()
        line = f.readline().decode('ascii')
        if is_valid(line):
          run = False        
        else:
          print("CRC8 failed. Invalid line!")
           
      Is = int(line.split(',')[6])
      v_rms = float(line.split(',')[7])   
      except:
        pass
  return [Is, v_rms]

def gpio_setup():
  gpio.setmode(gpio.BOARD)
  gpio.setup(35, gpio.OUT)
  gpio.output(35, gpio.HIGH)

def get_oscilloscope(instr):

    #instr.write(":STOP")
    # Votlage measurement
    instr.write(":MEAS:SOUR CHAN1")
    Vrms=float(instr.ask("MEAS:ITEM? PVRMS"))
    Freq=float(instr.ask("MEAS:ITEM? FREQ"))
    #cycles on screen
    c_os=100*6*1e-6*Freq

    instr.write(":MEAS:SOUR CHAN2")
    Imax=float(instr.ask("MEAS:VMAX?"))*1000   
    Irms=float(instr.ask("MEAS:ITEM? PVRMS"))*1000   

    if Imax>1e3:
        print('WARNING: Measured current is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR CHAN2")
        Imax=float(instr.ask("MEAS:VMAX?"))*1000   
        Irms=float(instr.ask("MEAS:VRMS?"))*1000   
    instr.write(":MEAS:SOUR MATH")
    P=float(instr.ask("MEAS:VAVG?"))
    Pp=float(instr.ask("MEAS:ITEM? MPAR"))   


    if P>1e3:
        print('WARNING: Measured power is too large')
        instr.write(":RUN")
        time.sleep(0.8)
        instr.write(":STOP")
        instr.write(":MEAS:SOUR MATH")
        P=float(instr.ask("MEAS:VAVG?"))
        Pp=float(instr.ask("MEAS:ITEM? MPAR"))   

   # instr.write(":RUN")
    time.sleep(0.4)
    return [Vrms,Imax,Irms,P,Pp,Freq]

def get_spec(spec):

    wv=spec.wavelengths()
    sp_int=spec.intensities()
    O777=max(sp_int[1200:1250])
    return O777

save_file=open('control_dat','a+')

#import input data
OL_opt=scio.loadmat('sweep_plasma_off.mat')
OL_in=OL_opt['u_opts']
Delta = 20 #how long each input combination is applied in s
osc_run=1;

X = []
U = []
Y = []


if __name__ == "__main__":
  """
  Collect data from the system, feed it into an MPC algorithm,
  solve an optimization problem, and then feed the results
  back into the system
  """

  runopts = get_runopts()
  # this does work, but it breaks the solver!
  #if runopts.quiet:
  #  ## silence the solver
  #  solver = nostdout(solver)
  gpio_setup()
  # shell calls - slow and somewhat brittle
  #f = subprocess.Popen(['tail','-f','./data/temperaturehistory'],\
  #        stdout = subprocess.PIPE,stderr=subprocess.PIPE)
  #p = select.poll()
  #p.register(f.stdout)
  # creating a good ol' serial object to read from when we want it
  f = serial.Serial('/dev/arduino', baudrate=38400,timeout=1)

  #initialize
  Tss = 0
  Ts = 0
  P = 0
  Ts_old = Ts 
  P_old = P
  first_run = 0
  delay = 1
  k = 0
  Y0 = 0
  startMPC = 0
  u_opt = [0,0,0]
  I1=0
  I2=0

  while True:
    Ts = get_temp(runopts)
    if Ts > 150:
        Ts=Ts_old
    else: 
        Ts_old=Ts

    [Is, v_rms] = get_intensity(f,runopts)

    [Vrms, Imax, Irms, P, Pp, Freq]= get_oscilloscope(instr)

    if P > 20:
        P=P_old
    else: 
        P_old=P

    O777=get_spec(spec)
   
    print("T(C): {:.2f}, I(a.u.): {:d}, Vrms(V): {:.2f}, Imax(mA): {:.2f}, Irms(mA): {:.2f}, Power(W):{:.2f},{:.2f}".format(Ts,Is,Vrms,Imax,Irms,P,Pp*Freq))


 
    # k is the loop instance (incremented each loop)
    if k == delay:
      ## set Y0 as the initial state
      #Y0 = [Ts,Is]
      Y0=[60,87]
      startMPC = True
      print("starting OL input sequence")
    elif k < delay:
      print("Don't start OL input sequence...")
      time.sleep(Delta)
      k=k+1
      start_time = time.time()
    else:
      print("OL input sequence is running")
    
    # The actual MPC part
    if startMPC:
      #if False:

      u_opt=OL_in[k-delay,:]
     

      U = u_opt

      Ureal = U + NP.array([U0['v'],U0['f'],U0['q']])
      #print(Ureal.shape)
      print("sending inputs...")
      send_inputs(f,U)
      print("inputs sent!")

      osc_run=1
      # figure out how long the loop took
      # if it's not time to run again, delay until it is
      end_time = time.time()
      time_el = end_time - start_time
      
       #save_file.write("{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(Ts,Is,*Y,*X,*U))
      save_file.write("{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f}\n".format(time.time(),Ts,Is,Vrms,v_rms,Imax,Irms,P,Pp*Freq,Freq/1000,O777,*U,time_el,k))  ##X is never referenced!
      #print()
      save_file.flush()






      if time_el < Delta:
        k = k
      else:
        k = k+1
        start_time = time.time()
      print("Next increment...")
    ## increment the loop counter
    
    print(k)
    print("\n\n")
