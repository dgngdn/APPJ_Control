
# Do not write bytecode to maintain clean directories
import sys
sys.dont_write_bytecode = True

# Imports required packages.
from casadi import *
import numpy as NP
import matplotlib.pyplot as plt
import scipy.io
from matplotlib.ticker import MaxNLocator
from scipy import linalg
from scipy import signal
from numpy import random
from scipy import io as sio
import matplotlib.pyplot as plt
import os
import datetime
import time
import cv2
import argparse
from pylepton import Lepton
import RPi.GPIO as gpio
gpio.setwarnings(False)
import subprocess
import select
import scipy.io as scio
import serial
import crcmod
import visa
sys.path.append('/home/brandon/repos/python-seabreeze')
import seabreeze.spectrometers as sb
import asyncio
import usbtmc
import socket
from scipy.interpolate import interp1d
# Import core code.
import core

#import model_v1 as jet
#import EKF_v1 as observer


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
    except Exception as e: # USBErrors
         print("{} in get_oscilloscope".format(e))
         time.sleep(0.4)
print("device info: {}".format(idg))
print("device timeout: {}".format(instr.timeout))
time.sleep(0.5)

## initialize spectrometer
devices = sb.list_devices()
#t_int=12000
t_int=12000*8
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
  """g
  Gets the arguments provided to the interpreter at runtime
  """
  parser = argparse.ArgumentParser(description="runs MPC",
			  epilog="Example: python mpc_lin_test.py --quiet")
  #parser.add_argument("--quiet", help="silence the solver", action="store_true")
  parser.add_argument("--faket", help="use fake temperature data", action="store_true")
  parser.add_argument("--fakei", help="use fake intensity data", action="store_true")
  parser.add_argument("--timeout", type=float, help="timout (seconds) on oscilloscope operations",
                            default=0.4)
  parser.add_argument("--save_therm", help="save thermography photos", action="store_true")
  parser.add_argument("--save_spec", help="save OES spectra", action="store_true")
  parser.add_argument("--dir", type=str, default="data",help="relative path to save the data")
  parser.add_argument("--tag", type=str, default="",help="tag the saved files for easy recognition")
  parser.add_argument("--auto",help="run the code without connection to laptop", action="store_true")
  runopts = parser.parse_args()
  return runopts

##define input zero point
#U0 = NP.array([(8.0,16.0,1.2,40)], dtype=[('v','>f4'),('f','>f4'),('q','>f4'),('d','>f4')])
runopts = get_runopts()
curtime1 = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")

SAVEDIR_therm = os.path.join(os.getcwd(),runopts.dir,"{}_thermography-{}".format(curtime1,runopts.tag)) # path to the directory to save thermography files
SAVEDIR_spec = os.path.join(os.getcwd(),runopts.dir,"{}_spectroscopy-{}".format(curtime1,runopts.tag)) # path to the directory to save thermograph


if runopts.save_therm and not os.path.exists(SAVEDIR_therm):
  print("Creating directory: {}".format(SAVEDIR_therm))
  os.makedirs(SAVEDIR_therm)

if runopts.save_spec and not os.path.exists(SAVEDIR_spec):
  print("Creating directory: {}".format(SAVEDIR_spec))
  os.makedirs(SAVEDIR_spec)


def save_data(SAVEDIR,data,fname):
  print("saving {}.csv".format(os.path.join(SAVEDIR,fname)))
  np.savetxt(os.path.join(SAVEDIR,"{}.csv".format(fname)),data,delimiter=',',fmt='%.4e')

def send_inputs_direct(device,U):
  """
  Sends input values to the microcontroller to actuate them
  """
  Vn = U[0]
  Fn = U[1]
  Qn = U[2]
  Dn = U[3]
  Xn = U[4]
  Yn = U[5]
  input_string='echo "v,{:.2f}" > /dev/arduino && echo "f,{:.2f}" > /dev/arduino && echo "q,{:.2f}" > /dev/arduino'.format(Vn, Fn, Qn)
  #subprocess.run('echo -e "v,{:.2f}\nf,{:.2f}\nq,{:.2f}" > /dev/arduino'.format(U[:,0][0]+8, U[:,1][0]+16, U[:,2][0]+1.2), shell=True)
  device.reset_input_buffer()
  device.write("v,{:.2f}\n".format(Vn).encode('ascii'))
  time.sleep(0.200)
  device.write("f,{:.2f}\n".format(Fn).encode('ascii'))
  time.sleep(0.200)
  device.write("q,{:.2f}\n".format(Qn).encode('ascii'))
  time.sleep(0.200)
  device.write("d,{:.2f}\n".format(Dn).encode('ascii'))
  time.sleep(0.400)
  device.write("x,{:.2f}\n".format(Xn).encode('ascii'))
  time.sleep(0.200)
  device.write("y,{:.2f}\n".format(Yn).encode('ascii'))
  time.sleep(0.200)

  print("input values: {:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}".format(Vn,Fn,Qn,Dn,Xn,Yn))

def send_inputs(device,U):
  """
  Sends input values to the microcontroller to actuate them
  """
  Vn = U[0]
  Fn = U[1]
  Qn = U[2]
  Dn = U[3]
  Xn = U[4]
  Yn = U[5]
  Pn = U[6]
  input_string='echo "v,{:.2f}" > /dev/arduino && echo "f,{:.2f}" > /dev/arduino && echo "q,{:.2f}" > /dev/arduino'.format(Vn, Fn, Qn)
  #subprocess.run('echo -e "v,{:.2f}\nf,{:.2f}\nq,{:.2f}" > /dev/arduino'.format(U[:,0][0]+8, U[:,1][0]+16, U[:,2][0]+1.2), shell=True)
  device.reset_input_buffer()

  subprocess.run('echo "" > /dev/arduino', shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "v,{:.2f}" > /dev/arduino'.format(Vn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "f,{:.2f}" > /dev/arduino'.format(Fn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "q,{:.2f}" > /dev/arduino'.format(Qn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "d,{:.2f}" > /dev/arduino'.format(Dn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "x,{:.2f}" > /dev/arduino'.format(Xn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "y,{:.2f}" > /dev/arduino'.format(Yn), shell=True)
  time.sleep(0.0500)

  subprocess.run('echo "p,{:.2f}" > /dev/arduino'.format(Pn), shell=True)
  time.sleep(0.0500)

  print("input values: {:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}".format(Vn,Fn,Qn,Dn,Xn,Yn,Pn))

def send_inputs_v_only(device,device2,Vn,Yn,Pn,Dn):
  """
  Sends input values to the microcontroller to actuate them
  """
  device.reset_input_buffer()

  subprocess.run('echo "" > /dev/arduino_m', shell=True)
  time.sleep(0.0500)
  subprocess.run('echo "w,{:.2f}" > /dev/arduino_m'.format(Vn), shell=True)
  time.sleep(0.0500)
  subprocess.run('echo "y,{:.2f}" > /dev/arduino_c'.format(Yn), shell=True)
  time.sleep(0.0500)
  subprocess.run('echo "d,{:.2f}" > /dev/arduino_c'.format(Dn), shell=True)
  time.sleep(0.0500)
  subprocess.run('echo "p,{:.2f}" > /dev/arduino_m'.format(Pn), shell=True)
  time.sleep(0.0500)
  print("input values: V:{:.2f},Y:{:.2f}".format(Vn,Yn))


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

def get_temp_max(runopts):
  """
  Gets treatment temperature with the Lepton thermal camera
  """
  if runopts.faket:
    return 24

  run = True
  for rr in range(8):
    try:
      with Lepton("/dev/spidev0.1") as l:
        data,_ = l.capture(retry_limit = 3)
      if l is not None:
         Ts = NP.amax(data[6:50,:,0]) / 100 - 273;
         Tt = NP.amax(data[0:5,:,0]) / 100 - 273;
         mm= NP.where( data == NP.amax(data[6:50,:,0]) )
         print('max point at {},{}'.format(*mm))
         for line in data:
            l = len(line)
            if (l != 80):
                print("error: should be 80 columns, but we got {}".format(l))
            elif Ts > 150:
                print("Measured temperature is too high: {}".format(Ts))
         time.sleep(0.070)
         run = False
    except Exception as e:
      print(e)
      print("\nHardware error on the thermal camera. Lepton restarting...")
      gpio.output(35, gpio.HIGH)
      time.sleep(0.5)
      gpio.output(35, gpio.LOW)
      print("Lepton restart completed!\n\n")

  return [int(mm[0]), int(mm[1])]
############################################ ASYNC DEFS ##################################################33

async def get_temp_a(runopts,a):
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
        mm=a            ### This may need calibration #############
        ##print(data[10:80]);
        Ts = NP.amax(data[6:50,:,0]) / 100 - 273;
        #Ts=data[int(mm[0]),int(mm[1]),0] / 100 - 273 #fixed_point_control
        #Ts=data[25,58,0]/100-273 #calibrated for the long jet
        #print(Ts_max, Ts)
        Tt = NP.amax(data[0:5,:,0]) / 100 - 273;
        #mm= NP.where( data == NP.amax(data[6:50,:,0]) )
        Ts_lin=data[int(mm[0]),:,0] /100 - 273
        yy=Ts_lin-Ts_lin[0]
        #gg=interp1d(yy,range(80))
        #sig=gg(0.6*NP.amax(yy))-mm[1]
        #print('sig',sig)
        Ts2 = (Ts_lin[int(mm[1])+2]+Ts_lin[int(mm[1])-2])/2
        Ts3 = (Ts_lin[int(mm[1])+12]+Ts_lin[int(mm[1])-12])/2
        Ts_lin_out=Ts_lin[int(mm[1])-13:int(mm[1])+13]
        for line in data:
          l = len(line)
          if (l != 80):
            print("error: should be 80 columns, but we got {}".format(l))
          elif Ts > 150:
            print("Measured temperature is too high: {}".format(Ts))

        if runopts.save_therm:
            curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
            fname = "{}".format(curtime)
            save_data(SAVEDIR_therm,data[:,:,0]/100.0 - 273 ,fname)

        if runopts.save_spec:
            wv=spec.wavelengths()
            sp_int=spec.intensities()
            data=[wv,sp_int]
            curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
            fname = "{}".format(curtime)
            save_data(SAVEDIR_spec,data,fname)

        #curtime = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S.%f")
        #fname = "{}".format(curtime)
        #Ts = NP.amax(data) / 100 - 273;
        #Ts = NP.true_divide(NP.amax(data[7:50]),100)-273;
        time.sleep(0.070)
        run = False
    except Exception as e:
      print(e)
      print("\nHardware error on the thermal camera. Lepton restarting...")
      gpio.output(35, gpio.HIGH)
      time.sleep(0.5)
      gpio.output(35, gpio.LOW)
      print("Lepton restart completed!\n\n")

  #print(Ts)
  return [Ts, Ts2, Ts3, Ts_lin_out, Tt, data]

async def get_intensity_a(f,f2,runopts):
  """
  Gets optical intensity from the microcontroller
  """
  if runopts.fakei:
    Is = 5
  else:
    run1 = True
    run2 = True
    v_rms=0
    Is=0
    U=[0,0,0]
    x_pos=0
    y_pos=0
    dsep=0
    T_emb=0
    while run1:
      try:
        f.reset_input_buffer()
        f.readline()
        line = f.readline().decode('ascii')
        if is_valid(line):
          run1 = False
          Is = int(line.split(',')[6])
          V_emb = float(line.split(',')[7])
          V = float(line.split(',')[1])
          f = float(line.split(',')[2])
          q = float(line.split(',')[3])
          Dc = float(line.split(',')[5])
          T_emb=float(line.split(',')[8])
          I_emb=float(line.split(',')[9])
          P_emb=float(line.split(',')[-2])
        else:
          print("CRC8 failed. Invalid line!")
      #    run = False
      #    Is = 0
      #    v_rms = 0
      #    V = 0
      #    f = 0
      #    q = 0

      #  U=[V,f,q,dsep]
        U=[V,f,q,dsep]
      except:
        pass
    while run2:
      try:
        f2.reset_input_buffer()
        f2.readline()
        line = f2.readline().decode('ascii')
        if is_valid(line):
          run2 = False
          dsep=float(line.split(',')[4])
          x_pos=float(line.split(',')[10])
          y_pos=float(line.split(',')[11])
        else:
          print("CRC8 failed. Invalid line!")
      #    run = False
      #    Is = 0
      #    v_rms = 0
      #    V = 0
      #    f = 0
      #    q = 0

        U=[V,f,q,dsep]
      except:
        pass

  #print(Is)
  return [Is,v_rms,U,x_pos,y_pos,dsep,T_emb,P_emb,Dc]

def gpio_setup():
  gpio.setmode(gpio.BOARD)
  gpio.setup(35, gpio.OUT)
  gpio.output(35, gpio.HIGH)

async def get_oscilloscope_a(instr):

   # instr.write(":STOP")
    # Votlage measurement
    instr.write(":MEAS:SOUR CHAN1")
    Vrms=float(instr.ask("MEAS:ITEM? PVRMS"))

    instr.write(":MEAS:SOUR CHAN2")
    Irms=float(instr.ask("MEAS:ITEM? PVRMS"))
    Imax=float(instr.ask("MEAS:VMAX?"))*1000 

    Prms=Vrms*Irms
    instr.write(":MEAS:SOUR MATH")
    P=float(instr.ask("MEAS:VAVG?"))

    if P>1e3:
        print('WARNING: Measured power is too large')
        time.sleep(0.8)
        instr.write(":MEAS:SOUR MATH")
        P=float(instr.ask("MEAS:VAVG?"))

    return [P,Imax,Prms]

async def get_spec_a(spec):

    wv=spec.wavelengths()
    sp_int=spec.intensities()
    sp_int=sp_int-NP.mean(sp_int[-20:-1])
    sum_int=NP.sum(sp_int[20:])
    O777=max(sp_int[1200:1250])
    O844=max(sp_int[1405:1455])
    N391=max(sp_int[126:146])
    He706=max(sp_int[990:1100])
    #print(O777)
    return [O777,O844,N391,He706,sum_int]


async def asynchronous_measure(f,instr,runopts,max_pt):

        tasks=[asyncio.ensure_future(get_temp_a(runopts,max_pt)),
              asyncio.ensure_future(get_intensity_a(f,f_move,runopts)),
              asyncio.ensure_future(get_oscilloscope_a(instr)),
              asyncio.ensure_future(get_spec_a(spec))]

        await asyncio.wait(tasks)
        return tasks


###########################################################

Ts_old=37
Ts2_old=32
Ts3_old=25
Tslin_old=[37]*26

Pold=2
runopts = get_runopts()
gpio_setup()
f = serial.Serial('/dev/arduino_m', baudrate=38400,timeout=1)
f_move = serial.Serial('/dev/arduino_c', baudrate=38400,timeout=1)
max_pt=get_temp_max(runopts) ##get maximum point

if os.name == 'nt':
    ioloop = asyncio.ProactorEventLoop() # for subprocess' pipes on Windows
    asyncio.set_event_loop(ioloop)
else:
    ioloop = asyncio.get_event_loop()

print(instr)


a=ioloop.run_until_complete(asynchronous_measure(f,instr,runopts,max_pt))

Temps=a[0].result()
Ts=Temps[0]
Ts2=Temps[1]
Ts3=Temps[2]
Ts_lin=Temps[3]
Tt=Temps[4]
sig_k=2*2.88/(NP.sqrt(-NP.log((Ts2-Ts3)/(Ts-Ts3))))

Ard_out=a[1].result()
Is=Ard_out[0]
v_rms=Ard_out[1]
x_pos=Ard_out[3]
y_pos=Ard_out[4]
d_sep=Ard_out[5]
T_emb=Ard_out[6]

Osc_out=a[2].result()
Vrms=Osc_out[0]
P=Osc_out[0]

delta=3.0
Ts_lin_old=Ts_lin
#print(Ts)
#print(P)
msg="Temperature: {:.2f} Power: {:.2f}".format(Ts,P)
print('Measurment working...')
CEM=np.zeros(np.shape(Ts_lin)) #initialize CEM measurement

print(msg)
if not runopts.auto:
    s=socket.socket(socket.SO_REUSEADDR,1)
    host='pi3.dyn.berkeley.edu'
    port=2223
    s.bind((host,port))
    s.listen(5)
    print('\n')
    print('Server is listening on port' ,port)
    print('Waiting for connection...')

    c, addr = s.accept()
    c.settimeout(0.2)

    print('Got connection from', addr)

#u_ub=[10.,20,4.,100.]
#u_lb=[6.,10.,1.2,100.]


u_ub=[6,20,4.,100.]
u_lb=[1,10.,1.2,10.]

#u_ub=[10,20,4.,100.]
#u_lb=[6,10.,1.2,10.]


V=2.#initial applied voltage
#V=9.5 #initial applied voltage8
q=1.5#initial flow
Dc=100 #initial duty cycle

############################### setpoints #####################
Tset=40 #initial setpoint
sigset=8.5 #initial setpoint
Pset=3.  #initial setpoint

######################################### set position parameters ####################
t_el=0  #seconds sup. control timer
tm_el=0
Y_pos=0.
#t_move=7.5 #seconds movement time
t_move=120.
#t_move=30.0
Delta_y=11. #mm
#_elps=0 #seconds movement timer
t_mel=0 #PI control timer
I1=0
I2=0
Dsep=4.0
Y_dir=-1

t_dis=100.0
t_step=120.
############ initialize jet position
send_inputs_v_only(f,V,q,Y_pos,Dc,Dsep)
print('initializing jet position...')
time.sleep(5.)

############ ini,  tialize save documents ################################



sv_fname = os.path.join(runopts.dir,"PI_Server_Out_{}-{}".format(curtime1,runopts.tag))
save_fl=open(sv_fname,'a+')
save_fl.write('time,Tset,Ts,Ts2,Ts3,P,Imax,O777,O845,N391,He706,sum_int,V,F,Q,D,x_pos,y_pos,T_emb,P_emb,t1\n')


t_move_now=time.time() ##movement_start
t_move_dis=time.time() ##disturbance_start
t_move_step=time.time() #step_start
while True:
    try:
        t0=time.time()
        try:

            data=c.recv(512).decode()
            data_str=data.split(',')
            data_flt=[float(i) for i in data_str]
            Tset=data_flt[0]
            t_el=data_flt[1]

            print('Optimal Reference Recieved!')
            print('Tref: {:6.2f}, t_el:{:6.2f}'.format(Tset,t_el))
        except:
            print('no data yet')


        a=ioloop.run_until_complete(asynchronous_measure(f,instr,runopts,max_pt))

        Temps=a[0].result()
        Ts_k=Temps[0]
        Ts2=Temps[1]
        Ts3=Temps[2]
        Ts_lin_k=Temps[3]
        Tt=Temps[4]
        sig=Temps[5]

        ## filter
        Ts=Ts*0.7+Ts_k*0.3
        Ts_lin=Ts_lin*0.7+Ts_lin_k*0.3
        
        Ard_out=a[1].result()
        Is=Ard_out[0]
        v_rms=Ard_out[1]
        U_m=Ard_out[2]
        x_pos=Ard_out[3]
        y_pos=Ard_out[4]
        d_sep=Ard_out[5]
        T_emb=Ard_out[6]
        P_emb=Ard_out[7]
        D_c=Ard_out[8]
       
        Osc_out=a[2].result()
        P=Osc_out[0]
        Imax=Osc_out[1]
        Prms=Osc_out[2]

        Spec_out=a[3].result()
        O777=Spec_out[0]
        O845=Spec_out[1]
        N391=Spec_out[2]
        He706=Spec_out[3]
        sum_int=Spec_out[-1]

     #   print("Temperature: {:.2f}, Sigma: {:.2f}, Power: {:.2f}".format(Ts,sig,P))
        print("Inputs:{:.2f},{:.2f},{:.2f},{:.2f}".format(*U_m))
        if abs(Ts)>90:
            Ts=Ts_old
            Ts2=Ts2_old
            Ts3=Ts3_old
            Ts_lin=Ts_lin_old
            print('WARNING: Old value of Ts is used')
        else:
            Ts_old=Ts
            Ts2_old=Ts2
            Ts3_old=Ts3
            Ts_lin_old=Ts_lin
        if abs(P)>10:
            P=Pold
            print('WARNING: Old value of Ts is used')
        else:
            Pold=P

        ########################## PI CONTROLS V=>T ################################
        Kp1=2.7
#        Tp1=28.8
        Tp1=15.0
        lamb1=40.0
#        lamb1=200.0

        Kc1=Tp1/(Kp1*lamb1)
        Ti1=Tp1
        e1=Tset-Ts

        u1= V+Kc1*(e1+I1/Ti1)

#        if round(u1,2)>=u_ub[0] and Tset>=Ts:
#            I1 = I1
#            V=u_ub[0]
#        elif round(u1,2)>=u_ub[0] and Tset<Ts:
#            I1 = I1 + e1*t_mel
#            V=u_ub[0]
#        elif round(u1,2)<=u_lb[0] and Tset<=Ts:
#            I1 = I1
#            V=u_lb[0]
#        elif round(u1,2)<=u_lb[0] and Tset>Ts:
#           I1 = I1 + e1*t_mel
#           V=u_lb[0]
#        else:
#            I1=I1+e1*t_mel
#            V=round(u1,2)

#        print(V)

        ########################## PI CONTROLS V=>P ################################
        Kp1=1.85
#        Tp1=28.8
        Tp1=1
        lamb1=2
#        lamb1=200.0

        Kc1=Tp1/(Kp1*lamb1)
        Ti1=Tp1
        e1=Pset-Prms

        u1= V+Kc1*(e1+I1/Ti1)

 #       if round(u1,2)>=u_ub[0] and Tset>=Ts:
 #           I1 = I1
 #           V=u_ub[0]
 #       elif round(u1,2)>=u_ub[0] and Tset<Ts:
 #           I1 = I1 + e1*t_mel
 #           V=u_ub[0]
 #       elif round(u1,2)<=u_lb[0] and Tset<=Ts:
 #           I1 = I1
 #           V=u_lb[0]
 #       elif round(u1,2)<=u_lb[0] and Tset>Ts:
 #          I1 = I1 + e1*t_mel
 #          V=u_lb[0]
 #       else:
 #           I1=I1+e1*t_mel
 #           V=round(u1,2)

        ########################## PI CONTROLS P=>T ################################
#        Kp1=4.6
        Kp1=9. #for metal
        Tp1=22.7
        lamb1=50.
        lamb1=200 #for metal

        Kc1=Tp1/(Kp1*lamb1)
        Ti1=Tp1
        e1=Tset-Ts

        u1= V+Kc1*(e1+I1/Ti1)
        
        if round(u1,2)>=u_ub[0] and Tset>=Ts:
            I1 = I1
            V=u_ub[0]
        elif round(u1,2)>=u_ub[0] and Tset<Ts:
            I1 = I1 + e1*t_mel
            V=u_ub[0]
        elif round(u1,2)<=u_lb[0] and Tset<=Ts:
            I1 = I1
            V=u_lb[0]
        elif round(u1,2)<=u_lb[0] and Tset>Ts:
           I1 = I1 + e1*t_mel
           V=u_lb[0]
        else:
            I1=I1+e1*t_mel
            V=round(u1,2)

#        print(V)
        ########################## PI CONTROLS Dcycle=>T ################################
        Kp1=0.25
        Tp1=20.4
        lamb1=50.
#        lamb1=200.0

        Kc1=Tp1/(Kp1*lamb1)
        Ti1=Tp1
        e1=Tset-Ts

        u1=Dc+Kc1*(e1+I1/Ti1)

   #     if round(u1,2)>=u_ub[3] and Tset>=Ts:
   #         I1 = I1
   #         Dc=u_ub[3]
   #     elif round(u1,2)>=u_ub[3] and Tset<Ts:
   #         I1 = I1 + e1*t_mel
   #         Dc=u_ub[3]
   #     elif round(u1,2)<=u_lb[3] and Tset<=Ts:
   #         I1 = I1
   #         Dc=u_lb[3]
   #     elif round(u1,2)<=u_lb[3] and Tset>Ts:
   #        I1 = I1 + e1*t_mel
   #        Dc=u_lb[3]
   #     else:
   #         I1=I1+e1*t_mel
   #         Dc=round(u1,2)

     ########################## PI CONTROLS Q=>T ################################
        Kp1=-4.0
        Tp1=12
        lamb1=120.0
#        lamb1=200.0

        Kc1=Tp1/(Kp1*lamb1)
        Ti1=Tp1
        e1=Tset-Ts

        u1= q+Kc1*(e1+I1/Ti1)

 #       if round(u1,2)>=u_ub[2] and Tset>=Ts:
 #           I1 = I1
 #           q=u_ub[2]
 #       elif round(u1,2)>=u_ub[2] and Tset<Ts:
 #          I1 = I1 + e1*t_mel
 #          q=u_ub[2]
 #       elif round(u1,2)<=u_lb[2] and Tset<=Ts:
 #           I1 = I1
 #           q=u_lb[2]
 #       elif round(u1,2)<=u_lb[2] and Tset>Ts:
 #          I1 = I1 + e1*t_mel
 #          q=u_lb[2]
 #       else:
 #           I1=I1+e1*t_mel
 #           q=round(u1,2)

#        print(V)
        ########################### PI CONTROLS Q=>SIGMA ###########################3
        Kp2=4.943/9.164
        Tp2=1/9.164
        lamb2=1

        Kc2=Tp2/(Kp2*lamb2)
        Ti2=Tp2
        e2=sigset-sig

        u2=q+Kc2*(e2+I2/Ti2)


    #    if round(u2,2)>=u_ub[2] and sigset>=sig:
    #        I2 = I2
    #        q=u_ub[2]
    #    elif round(u2,2)>=u_ub[2] and sigset<sig:
    #        I2 = I2 + e1*t_mel
    #        q=u_ub[2]
    #    elif round(u2,2)<=u_lb[2] and sigset<=sig:
    #        I2 = I2
    #        q=u_lb[2]
    #    elif round(u2,2)<=u_lb[2] and sigset>sig:
    #       I2 = I2 + e2*t_mel
    #       q=u_lb[2]
    #    else:
    #        I2=I2+e2*t_mel
    #        q=round(u2,2)


        ########################### Movement ##############################

        if Y_pos==0: #for case I and II
            Y_dir=-1
        if Y_pos==-11: #for case I and II
            Y_dir=1
        if Y_dir==1 and (time.time()-t_move_now)>=t_move: #for case I and II
                print('Moving')
                Y_pos=Y_pos+Delta_y
                t_move_now=time.time()
        elif Y_dir==-1 and (time.time()-t_move_now)>=t_move: #for case I and II
                print('Moving')
                Y_pos=Y_pos-Delta_y
                t_move_now=time.time()

###### DISTURBANCE
#        if (time.time()-t_move_dis)>=t_dis:
#            print('Moving')
#            Dsep=Dsep+2
#            t_move_dis=time.time()

#        if (time.time()-t_move_dis)>=t_dis:
#            print('Moving')
#            if Dsep==4.:
#               Dsep=6.
#            else:
#                Dsep=4.
#            t_move_dis=time.time()

          ####################### STEP TEST ########################################
#        if (time.time()-t_move_step)>=t_step:
#          Tset=Tset+1
#          t_move_step=time.time()

        ######################### Send inputs #########################
        print("Sending inputs...")
        send_inputs_v_only(f,f_move,V,Y_pos,Dc,Dsep)
        print("Inputs sent!")

        ##interpolate temperature to shift position
        x_gen=range(26) #range of points controlled  [0-25mm]
        x_now=NP.linspace(-13.0*2.89,12.0*2.89,26)+Y_pos #positions corresponding to current measurement


        Tshift=interp1d(x_now,Ts_lin,bounds_error=False,fill_value=min(Ts_lin))(x_gen)
        print(Ts_lin)
        CEM=CEM+tm_el*(9.74e-14/60.0)*np.exp(np.multiply(0.6964,Tshift))
        msg='{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f} \n'.format(*CEM)

        print(msg)

        if not runopts.auto:
            c.send(msg.encode())
            print('Measured outputs sent')

        save_fl.write('{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f},{:6.2f}\n'.format(time.time(),Tset,Ts,Ts2,Ts3,P,Imax,O777,O845,N391,He706,sum_int,*U_m,x_pos,y_pos,T_emb,V,P_emb,Prms,D_c,tm_el))
        save_fl.flush()


        tm_el=time.time()-t0

        if tm_el<1.3:
            time.sleep(1.3-tm_el)
            tm_el=1.3

        if KeyboardInterrupt==1:
            sys.exit(1)

    except Exception as e:
        if not runopts.auto:
            c.close()
        print('There was an error !')
        print(e)
        sys.exit(1)
