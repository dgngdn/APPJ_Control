

## References

+ [Protek 9301 Function Generator Manual][manual]
+ [Wikipedia: Newline][wiki-newline]

Terminal settings:

1) Synchronization: ASYNC
2) Stop Bit: 2
3) Parity: None
4) Data Length: 8bit
5) Communication Speed: 2400, 4800, 9600, 19200 BPS

[manual]: https://docs.google.com/file/d/0B-CoZGWGk5pvclBNc0J5RFpZVE0/edit
[wiki-newline]: https://learn.sparkfun.com/tutorials/raspberry-gpio

## Remote Controls

A0	set sine
A1	set square
A2	set triangle
A3	set ramp

B0	set AM mode modulation

C0	set sine mode modulation
C1	set square mode modulation
C2	set triangle mode modulation
C3	set ramp mode modulation
C4	set single mode modulation
c5	set arbitrary mode modulation
c0?	get setting mode modulation

D0	set modulation off
D1	set modulation on
D0?	gets modulation mode

E0	set frequency			MHz ✓
E>	set	amplitude level		Vpp ✓
E?	set amplitude level		Vrms ✓

EB	set amplitude depth		% (of modulation) ✓
E;	set modulation rate		kHz ✓


## Modulation

Modulation Type:		AM
Modulation Span:		100%
Modulation Waveform:	Square
Modulation Rate:		1 kHz?

the modulating square wave duty cycle CANNOT be changed
so, you'd need to load in arbitrary waveforms of square waves with different duty cycles...?


## Hardware

Bus 001 Device 022: ID 0557:2008 ATEN International Co., Ltd UC-232A Serial Port [pl2303]


## Software

nano asd

    #!/usr/bin/env bash
    stty -F /dev/ttyUSB0 cs8 cstopb -parity 9600
    printf "$1\n" > /dev/ttyUSB0

chmod a+x asd
sudo nano /etc/udev/rules.d/protek.rules

    # Bus 001 Device 022: ID 0557:2008 ATEN International Co., Ltd UC-232A Serial Port [pl2303]
    SUBSYSTEMS=="tty", ACTION=="add", ATTRS{idVendor}=="0557", ATTRS{idProduct}=="2008", GROUP="dialout", MODE="0660", SYMLINK+="protek"

sudo udevadm control --reload
sudo udevadm trigger

    ./asd "E>10"
    ./asd "E00.020000"

