#!/usr/env python

import os
import subprocess
import serial
import datetime
import time
import numpy as np
import cv2
from pylepton import Lepton


DATA_ELEMENTS = 2
DATA_LENGTH = 13

arduino = serial.Serial("/dev/arduino", baudrate=9600, timeout=1.5)
expfolder = datetime.datetime.isoformat(datetime.datetime.today())[:16]
FNULL = open(os.devnull, 'w')

subprocess.check_call(['mkdir',expfolder])
print("saving data in {}...".format(expfolder))

def dataquality(data):
    delements = len(data.split(','))
    dlength = len(data)
    if (delements == DATA_ELEMENTS and dlength == DATA_LENGTH):
        return True
    else:
        #print("elements: {}".format(delements))
        #print("length: {}".format(dlength))
        return False

while True:
    arduino.reset_input_buffer()
    with open(os.path.join(expfolder,'thermopile'),'a') as f, open(os.path.join(expfolder,'thermograph'),'a') as g:
        #subprocess.call(['./rascap'], stdout=FNULL, cwd=os.path.join(expfolder))
        with Lepton("/dev/spidev0.1") as l:
            thermograph,_ = l.capture()
        for line in thermograph:
            l = len(line)
            if (l != 80):
                print(l)
        ts = int(1000*time.time())
        print("thermograph captured at {}...".format(ts))
        g.write("{},".format(ts))
        np.savetxt(g, thermograph, delimiter=",",newline=",",fmt='%5d')
        g.write("\n")

        data = arduino.readline()
        ts = int(1000*time.time())
        if dataquality(data):
            f.write("{},{}".format(ts,data))
            f.flush()
        time.sleep(1)
