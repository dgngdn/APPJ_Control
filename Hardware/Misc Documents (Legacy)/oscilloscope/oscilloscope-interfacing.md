
idVendor
The hex number 0957 converts to 2391 in Decimal

idProduct
1755 = 5973



-----
ID 1ab1:04ce Rigol Technologies

6833
1230

pip install pyusb python-usbtmc
----
>>> import usbtmc
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/brandon/Dev/graveslab/local/lib/python2.7/site-packages/usbtmc/__init__.py", line 29, in <module>
    from .usbtmc import Instrument
  File "/home/brandon/Dev/graveslab/local/lib/python2.7/site-packages/usbtmc/usbtmc.py", line 27, in <module>
    import usb.core
ImportError: No module named usb.core

import usbtmc
instr =  usbtmc.Instrument(6833,1230)
print(instr.ask("*IDN?"))

----
sudo apt-get install libusb-dev

brandon@D105:~$ dpkg -l | grep libusb
ii  libgusb2:amd64                                              0.1.6-5                                             amd64        GLib wrapper around libusb1
ii  libusb-0.1-4:amd64                                          2:0.1.12-23.3ubuntu1                                amd64        userspace USB programming library
ii  libusb-1.0-0:amd64                                          2:1.0.17-1ubuntu2                                   amd64        userspace USB programming library
ii  libusb-1.0-0:i386                                           2:1.0.17-1ubuntu2                                   i386         userspace USB programming library
ii  libusb-dev                                                  2:0.1.12-23.3ubuntu1                                amd64        userspace USB programming library development files
ii  libusbmuxd2                                                 1.0.8-2ubuntu1                                      amd64        USB multiplexor daemon for iPhone and iPod Touch devices - library
ii  libusbredirparser1:amd64                                    0.6-2ubuntu1.1                                      amd64        Parser for the usbredir protocol (runtime)
----

python-ivi/python-usbtmc


00:19:AF:34:3A:F4
00-19-AF-34-3A-F4

Welcome to Web of DS1000Z Series
Information About This Instrument:
Instrument Model:	DS1104Z
Manufacturer:	RIGOL TECHNOLOGIES
Serial Number:	DS1ZA164457681
Description:	rigollan
LXI Class:	LXI Core 2011
LXI Version:	1.4
Host Name:	rigollan.local
MAC Address:	00-19-AF-34-3A-F4
IP Address:	169.229.198.103
Firmware Revision:	00.04.02.SP3
VISA TCP/IP String:	TCPIP::169.229.198.103::INSTR
Auto-MDIX Capable:	NO
VISA USB Connect String:	USB0::0x1AB1::0x4CE::DS1ZA164457681::INSTR

----
http://stackoverflow.com/questions/31564583/usbtmc-in-python

I used this guide:
https://xdevs.com/guide/ni_gpib_rpi/

SUCCESS over VXI11 in Python!

>>> import vxi11
>>> instr = vxi11.Instrument("169.229.198.103")
>>> print(instr.ask("*IDN?"))
RIGOL TECHNOLOGIES,DS1104Z,DS1ZA164457681,00.04.02.SP3






