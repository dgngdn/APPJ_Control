# -*- coding: utf-8 -*-
"""
Created on Tue Nov 07 12:11:51 2017

@author: Dogan
"""
import socket
import time
import sys
sys.path.append("C:\\Users\\Dogan\\Documents\\Casadi_Python")
sys.path.append("C:\\Users\\Dogan\\Dropbox\\COMSOL\\Flow_model")
sys.dont_write_bytecode = True
import pyximport; pyximport.install()

# Imports required packages.
from casadi import *
import numpy as NP
import matplotlib.pyplot as plt
import scipy.io as sio
#from matplotlib.ticker import MaxNLocator
from scipy import linalg
#from scipy import signal
#from numpy import random
from scipy import io as sio
import matplotlib.pyplot as plt
#import control
import datetime
from scipy.interpolate import interp1d
import model_v1_1 as jet
import EKF_v1 as observer
# Import core code.
import core
import json
############# INITIALIZE EVERYTHING ##################
delta_c=1.3# seconds time discretization
T = 300.0 # Time horizon in seconds
x_disc=26 # discretization of spatial
x_scl=26/float(x_disc)
N=int(T/delta_c)
nx=x_disc # state dimension

##load info
#r=sio.loadmat('opt_setpoint_case1.mat')#

#r=sio.loadmat('opt_setpoint_withTset_case3.mat')

#Initialize parameters
u_ub = [45] #input upper bound
u_lb = [30] #output lower bound
# t_move=int(60.0/delta) #movement time in sample instants



D_sep=4.0
X_pos=0.0
Dcycle=100
dis=[0,0]
Ac=7.0686e-06
t_elps=0
t_dis=0
dis_counter=0
t_el=0.0
Y_dir=1

####################### GENERATE POSITION VECTOR ################################
Y_pos=6.0
t_move=int(60.0/delta_c)
t_move=100 #secons 300/3
Delta_x=7.##mm
t_elps=0
Pos_g=[]

#### Generate position vector for case I and II
t_pos=np.arange(0,T,1)
for i in range(int(T)):
    t_elps=t_elps+1
    if t_elps==t_move:
        Y_pos=Y_pos+Delta_x
        t_elps=0
    Pos_g+=[Y_pos]

t_ocp=np.arange(0,T,delta_c)
pos_int=interp1d(t_pos,Pos_g,bounds_error=False, fill_value=max(Pos_g))

Pos=pos_int(t_ocp)

##### Generate position vector for case III
# Y_dir=1
# for i in range(N):
#     if Y_pos==18:
#         Y_dir=-1
#     elif Y_pos==0:
#         Y_dir=1 
#           
#     if Y_dir==1:
#         Y_pos=Y_pos+0.5
#     elif Y_dir==-1:
#         Y_pos=Y_pos-0.5
#     Pos+=[Y_pos]

# ###################### FORMULATE SYSTEM DYN #########################################
# 
# # Declare model variables
# x = MX.sym('x',x_disc)
# u = MX.sym('u',2)
# sigma=9.7
# 
# # Model equations
# xdot=(9.74e-14/60.0)*np.exp(0.6964*((u[0]-23)*np.exp(-((0-u[1])/sigma)**2)+23))
# 
# for i in range(x_disc-1):
#     xdot= vertcat(xdot,(9.74e-14/60.0)*np.exp(0.6964*((u[0]-23)*np.exp(-(((i+1)*x_scl-u[1])/sigma)**2)+23)))
# 
# 
# # CVODES from the SUNDIALS suite
# dae = {'x':x, 'p':u, 'ode':xdot}
# opts = {'tf':delta_c}
# 
# F = integrator('F', 'cvodes', dae, opts)
#    
# # Evaluate at a test points
# Fk = F(x0=np.zeros(x_disc),p=[43,3])
# print(Fk['xf'])
# print(Fk['qf'])
# 
# Fk = F(x0=np.zeros(x_disc),p=[43,3])
# print(Fk['xf'])
# print(Fk['qf'])
# 
# 
# ################### FORMULATE NLP FIRST TIME #####################
# w=[]
# w0 = []
# lbw = []
# ubw = []
# J = 0
# g=[]
# lbg = []
# ubg = []
# 
# # "Lift" initial conditions
# X0 = MX.sym('X0', x_disc)
# Xk = X0
# w += [X0]
# lbw += [0]*x_disc
# ubw += [0]*x_disc
# w0 += [0]*x_disc
# U0=42
# 
# for k in range(N):
#     # New NLP variable for the control
#     Uk = MX.sym('U_' + str(k),1)
#     w   += [Uk]
#     lbw += u_lb
#     ubw += u_ub
#     w0  += [42]
#     
#     J=J + 10*((U0-Uk))**2
#     U0=Uk
#     #J=J+(Uk[0]-42)**2
# 
#     # Integrate till the end of the interval
#     Fk = F(x0=Xk, p=vertcat(Uk,Pos[k]))
#     Xk_end = Fk['xf']    
# 
# 
#     # New NLP variable for state at end of interval
#     Xk = MX.sym('X_' + str(k+1), x_disc)
#     w   += [Xk]
#     lbw += [0]*x_disc
#     ubw += [inf]*x_disc
#     w0  += [0]*x_disc
# 
#     # Add equality constraint
#     g   += [Xk_end-Xk]
#     lbg += [0]*x_disc
#     ubg += [0]*x_disc
# 
# 
# for i in range(int(3/x_scl)):
#     J=J+ 100.0*(Xk[i]-0)**2
#     
# for i in range(int(3/x_scl),int(23/x_scl)):
#     J=J+ 100.0*(Xk[i]-1)**2
# 
# for i in range(int(23/x_scl),int(x_disc)):
#     J=J+ 100.0*(Xk[i]-0)**2
#     
# # Create an NLP solver
# prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
# solver = nlpsol('solver', 'ipopt', prob);
# 
# ############# SOLVE NLP FIRST TIME #############################
# 
# try: 
#     sol = solver(x0=r['w_opt'][0], lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
#     w_opt = sol['x'].full().flatten()
# except:
#     sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
#     w_opt = sol['x'].full().flatten()
#    
# seq_opt=w_opt[x_disc::x_disc+1] ## first element of optimal setpoint trajectory

##########################################################################
with open('caseIII_offline_optimal_w_constr(1).txt', 'r') as file:
     sv_dict=json.loads(file.read()) # use `json.loads` to do the reverse
 
seq_opt=sv_dict['T_opt']   

#initialize storage 
Y_STORE=NP.array([0]*x_disc)
REF_STORE=NP.array([0])
TIME=NP.array([time.time()])

############## SETUP TCP/IP SOCKET CLIENT######################################
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = 'pi3.dyn.berkeley.edu'
print (host)
port = 2223


s.connect((host, port))
print('Connection established...')

u_opt=seq_opt[0]
U=vertcat(u_opt,t_el)
msg = ','.join(str(e) for e in U.full().flatten())
print('Sending initial point...')
s.send(msg)


############## LET SYS SETTLE ######################################
# 
# M=5
# #let observer settle
# for m in range(M):
# 
#     print('Initializing...{:.2f}s remaining'.format((M-m)*delta))
#     a=s.recv(512).decode() #recieve measurement
#     #print(a) #print
# 
#     y_meas=[ [i] for i in a.split('\n')] #convert to float
#     y_m=[float(k) for k in y_meas[-2:][0][0].split(',')]
#     print(y_m)
#     s.send(msg)
# 
# D_sep=4.0
# 
# Tref=r['T_opt'][0]
# x_disc=25 # discretization of spatial
# Tref=r['w_opt'][0][x_disc::x_disc+1]
# Posy=r['Pos'][0]

########################### MAIN LOOP #########################################
t_0=time.time()
t_el=time.time()-t_0
meas_flag=1
for rr in range(len(seq_opt)+5):
    t_now=time.time()
    ##get measurement
        
    a=s.recv(1024).decode() #recieve measurement     
    y_meas=[ [i] for i in a.split('\n')] #convert to float
    y_m=[float(k) for k in y_meas[-2:][0][0].split(',')]

    t_el=time.time()-t_0   ## calculate remaining time
    try:
        u_opt=seq_opt[rr]
    except:
        u_opt=u_opt
########### Send Messege
    #u_opt=42.##CONSTANT SETPOINT
    U=vertcat(u_opt,t_el)
    msg = ','.join(str(e) for e in U.full().flatten())
    s.send(msg)
    print(msg)
    print(t_el)
  
############# Update the storage matrices
              
    Y_STORE=NP.vstack((Y_STORE,NP.array(y_m).T))
    REF_STORE=NP.vstack((REF_STORE,NP.array(U.full().flatten()[0])))
    TIME=NP.vstack((TIME,NP.array(time.time())))
    t_el=time.time()-t_0
    
    if time.time()-t_now<delta_c:
        time.sleep(delta_c-(time.time()-t_now))
        
s.close()
export_dict = {
    "Y_STORE":Y_STORE,
    "REF_STORE":REF_STORE,
    "POS_STORE":sv_dict['Pos'],
    "TIME":TIME,
    }

name_str='{:%Y-%m-%d_%H-%M-%S}_Supervisory_Control_Output'.format(datetime.datetime.now())

sio.savemat(name_str, mdict=export_dict)
