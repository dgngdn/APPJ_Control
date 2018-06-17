

# Standard Transports and Protocols

+ https://en.wikipedia.org/wiki/Virtual_Instrument_Software_Architecture
+ https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments
+ https://en.wikipedia.org/wiki/VME_eXtensions_for_Instrumentation


# What We're Using

## PyVISA-py

https://pyvisa-py.readthedocs.io/
https://github.com/hgrecco/pyvisa-py

"PyVISA-py is a backend for PyVISA. It implements most of the methods for Message Based communication (Serial/USB/GPIB/Ethernet) using Python and some well developed, easy to deploy and cross platform libraries."


## PyVISA

https://pyvisa.readthedocs.io/en/stable/

"PyVISA is a Python package that enables you to control all kinds of measurement devices independently of the interface (e.g. GPIB, RS232, USB, Ethernet)."





# Alternatives

## python-usbtmc

+ https://github.com/python-ivi/python-usbtmc/
+ https://pypi.python.org/pypi/python-usbtmc python-usbtmc 0.8
+ http://alexforencich.com/wiki/en/python-usbtmc/start


`pip install pyusb python-usbtmc`

import numpy as np
import usbtmc
import time

i = usbtmc.Instrument(0x1ab1, 0x04ce)
time.sleep(1)
i.timeout = 1.0 # this is in SECONDS
i.rigol_quirk_ieee_block = False
i.ask('*IDN?')
vars(i)
i.write(":WAV:CHAN1")
i.write(":WAV:DATA?")
a = np.fromstring(i.read_raw()[11:],dtype=float,sep=',')

i.write(":RUN")
i.ask(":TRIGGER:STATUS?")
i.write(":STOP")
i.ask(":TRIGGER:STATUS?")



It has some code to deal with the "Rigol quirk":

    RIGOL_QUIRK_PIDS = [0x04ce, 0x0588]

The 0x04ce (DS1000Z) device also has self.rigol_quirk_ieee_block = True, which has been fixed in the newest firmware (00.04.03.02.03); set this to False to get python-usbtmc working again.


## usbtmc.c

http://lxr.free-electrons.com/source/drivers/usb/class/usbtmc.c


It also has some code to deal with the "Rigol quirk":

    125 struct usbtmc_ID_rigol_quirk {
    126         __u16 idVendor;
    127         __u16 idProduct;
    128 };
    129
    130 static const struct usbtmc_ID_rigol_quirk usbtmc_id_quirk[] = {
    131         { 0x1ab1, 0x0588 },
    132         { 0x1ab1, 0x04b0 },
    133         { 0, 0 }
    134 };


## Rigol DS1000Z IVI Driver for Windows

http://www.rigol.eu/products/digital-oscilloscopes/ds1000z/



## Raw File Read/Write with Linux usbtmc.ko kernel module

import os
device = os.open("/dev/usbtmc0",os.O_RDWR)

while True:
    os.write(device,":WAV:DATA?")
    os.read(device,4000)
