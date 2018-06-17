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

############# INITIALIZE EVERYTHING ##################
delta_c=6.0# seconds time discretization
T = 312. # Time horizon in seconds
#T = 690. # Time horizon in seconds
x_disc=26 # discretization of spatial
x_scl=26/float(x_disc)
N=int(T/delta_c)
nx=x_disc # state dimension

r=sio.loadmat('opt_setpoint_case3_2.mat')

#Initialize parameters
u_ub = [45] #input upper bound
u_lb = [34] #output lower bound
# t_move=int(60.0/delta) #movement time in sample instants


#initialize storage 
Y_STORE=NP.array([0]*x_disc)
REF_STORE=NP.array([0])
TIME=NP.array([time.time()])

D_sep=4.0
X_pos=0.0
Dcycle=100
dis=[0,0]
Ac=7.0686e-06
t_elps=0
t_dis=0
dis_counter=0
t_el=0.0

## weights
Qdel=5 #nominal value of 5
Qcem=200.0 #nominal value of 100

####################### GENERATE POSITION VECTOR ################################
Y_dir=1
Y_pos=3.0
#t_move=30.
t_move=1.3*6. #seconds 300/10
Delta_y=1.##mm
t_elps=0
Pos_g=[]

#### Generate position vector for case I and II
t_pos=np.arange(0,T,1)
for i in range(int(T)):
    if Y_pos>=23: #for case I and II
        Y_dir=-1
    if Y_pos<=3: #for case I and II
        Y_dir=1

    if Y_dir==1 and t_elps>=t_move: #for case I and II
       Y_pos=Y_pos+Delta_y
       t_elps=0
    elif Y_dir==-1 and t_elps>=t_move: #for case I and II
       Y_pos=Y_pos-Delta_y
       t_elps=0
       
    t_elps=t_elps+1
    Pos_g+=[Y_pos]

t_ocp=np.arange(0,T,delta_c)
pos_int=interp1d(t_pos,Pos_g,bounds_error=False, fill_value=max(Pos_g))

Pos=pos_int(t_ocp)

###################### FORMULATE SYSTEM DYN #########################################

# Declare model variables
x = MX.sym('x',x_disc)
u = MX.sym('u',2)
sigma=7.5

# Model equations
xdot=(9.74e-14/60.0)*np.exp(0.6964*((u[0]-23)*np.exp(-((0-u[1])/sigma)**2)+23))

for i in range(x_disc-1):
    xdot= vertcat(xdot,(9.74e-14/60.0)*np.exp(0.6964*((u[0]-23)*np.exp(-(((i+1)-u[1])/sigma)**2)+23)))


# CVODES from the SUNDIALS suite
dae = {'x':x, 'p':u, 'ode':xdot}
opts = {'tf':delta_c}

F = integrator('F', 'cvodes', dae, opts)
   
# Evaluate at a test points
Fk = F(x0=np.zeros(x_disc),p=[43,3])
print(Fk['xf'])
print(Fk['qf'])

Fk = F(x0=np.zeros(x_disc),p=[43,3])
print(Fk['xf'])
print(Fk['qf'])


################### FORMULATE NLP FIRST TIME #####################
w=[]
w0 = []
lbw = []
ubw = []
J = 0
g=[]
lbg = []
ubg = []

# "Lift" initial conditions
X0 = MX.sym('X0', x_disc)
Xk = X0
w += [X0]
lbw += [0]*x_disc
ubw += [0]*x_disc
w0 += [0]*x_disc
U0=41

for k in range(N):
    # New NLP variable for the control
    Uk = MX.sym('U_' + str(k),1)
    w   += [Uk]
    lbw += u_lb
    ubw += u_ub
    w0  += [42]
    
    J=J + Qdel*((U0-Uk))**2
    U0=Uk
    #J=J+(Uk[0]-42)**2

    # Integrate till the end of the interval
    Fk = F(x0=Xk, p=vertcat(Uk,Pos[k]))
    Xk_end = Fk['xf']    


    # New NLP variable for state at end of interval
    Xk = MX.sym('X_' + str(k+1), x_disc)
    w   += [Xk]
    lbw += [0]*x_disc
    ubw += [1.1]*x_disc #hard cosntraint on initial solution
    w0  += [0]*x_disc
    
    # Add equality constraint
    g   += [Xk_end-Xk]
    lbg += [0]*x_disc
    ubg += [0]*x_disc


for i in range(int(3/x_scl)):
    J=J+ Qcem*(Xk[i]-0)**2+ Qcem/2.*(Xk[i]-0)
    
for i in range(int(3/x_scl),int(23/x_scl)):
    J=J+ Qcem*(Xk[i]-1)**2+ Qcem/2.*(Xk[i]-1)

for i in range(int(23/x_scl),int(x_disc)):
    J=J+ Qcem*(Xk[i]-0)**2+ Qcem/2.*(Xk[i]-0)
    
# Create an NLP solver
prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
solver = nlpsol('solver', 'ipopt', prob);

############# SOLVE NLP FIRST TIME #############################

# try: 
#     sol = solver(x0=r['w_opt'][0], lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
#     w_opt = sol['x'].full().flatten()
# except:
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()
    
u_opt=w_opt[x_disc] ## first element of optimal setpoint trajectory

############## SETUP TCP/IP SOCKET CLIENT######################################
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = 'pi3.dyn.berkeley.edu'
print (host)
port = 2223

s.connect((host, port))
print('Connection established...')


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
while t_el<T:
    t_now=time.time()
    ##get measurement
    while meas_flag==1:
        try:
            a=s.recv(1024).decode() #recieve measurement     
            y_meas=[ [i] for i in a.split('\n')] #convert to float
            y_m=[float(k) for k in y_meas[-2:][0][0].split(',')]
            meas_flag=0
        except:
            print('o-oh')
    
    meas_flag=1
    t_el=time.time()-t_0   ## calculate remaining time
    N=int((T-t_el)/delta_c) ## calculate the horizon length
    
    t_ocp_n=np.arange(round(t_el,2),T,delta_c)
    Pos=pos_int(t_ocp_n)
    N=len(Pos)
    #### formulate control problem ####################################
    # Start with an empty NLP
    w=[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []
    
    # "Lift" initial conditions
    X0 = MX.sym('X0', x_disc)
    Xk = X0
    w += [X0]
    lbw += y_m
    ubw += y_m
    w0 += y_m
    U0=u_opt

  
    for k in range(N):
        # New NLP variable for the control
        Uk = MX.sym('U_' + str(k),1)
        w   += [Uk]
        lbw += u_lb
        ubw += u_ub
        w0  += [42]
        
        J=J + Qdel*((U0-Uk))**2
        U0=Uk
    
        # Integrate till the end of the interval
        Fk = F(x0=Xk, p=vertcat(Uk,Pos[k]))
        Xk_end = Fk['xf']    
    
    
        # New NLP variable for state at end of interval
        Xk = MX.sym('X_' + str(k+1), x_disc)
        w   += [Xk]
        lbw += [0.]*x_disc
        ubw += [inf]*x_disc
    
        # Add equality constraint
        g   += [Xk_end-Xk]
        lbg += [0.]*x_disc
        ubg += [0.]*x_disc
        
    # ubw[-int(x_disc):]=[1.1]*x_disc #hard constr
    
    
    for i in range(int(3/x_scl)):
        J=J+ Qcem*(Xk[i]-0)**2#+Qcem/2.*(Xk[i]-0)
        
    for i in range(int(3/x_scl),int(23/x_scl)):
        J=J+ Qcem*(Xk[i]-1)**2#+Qcem/2.*(Xk[i]-1)
    
    for i in range(int(23/x_scl),int(x_disc)):
        J=J+ Qcem*(Xk[i]-0)**2#+Qcem/2.*(Xk[i]-0)
        
        
    # Create an NLP solver
    sol_opts={'ipopt.print_level':1, 'ipopt.max_cpu_time':10}
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob,sol_opts);
    ## warm start
    q0=vertcat(y_m,w_opt[-(N*(x_disc+1)):])
    
    ##solve
  
    sol = solver(x0=q0.full(), lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()
    u_opt=w_opt[x_disc] ## first element of optimal setpoint trajectory
  
        
########### Send Messege

    U=vertcat(u_opt,t_el)
    msg = ','.join(str(e) for e in U.full().flatten())
    s.send(msg)
    print(msg)
  
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
    "POS_STORE":Pos,
    "TIME":TIME,
    }

name_str='{:%Y-%m-%d_%H-%M-%S}_Supervisory_Control_Output'.format(datetime.datetime.now())

#sio.savemat(name_str, mdict=export_dict)
