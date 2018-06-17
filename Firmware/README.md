## Firmware Version
Most up-to date version of the Firmware is V14 including two flow controls and embedded power control. In V14, applied voltage is no longer an input parameter.
The older version V12 features an embedded controller for applied voltage at the electrode allowing for direct manipulation of the applied voltage.

## Serial configuration
Information can be exchanged with the Arduino through the Arduino IDE or the Linux terminal on Raspberry Pi. To configure the arduino run

```
~$stty -F /dev/[ARD_ADDR] raw 38400 –hupcl
```

here `[ARD_ADDR]` refers to the adress of the device. You can configure a custom name for your Arduino using udev-rules. By defult the 
Arduino adress is of form `ttyACM0`. Once configured you can read from the serial port with
```
~$cat /dev/[ARDUINO_ADRESS]
```

## Sending Commands

Arduino firmware is configured to accept commands of form `[char],[value]`. Where a characther determines what input is manipulated and the value
determines the input magnitude. As of v14 arduino accepts the following commands

| Input | Characther  | Range         | Example |
| ------------- |---------| -----|-----|
| Duty Cycle   | p | 0-100 (%) |`$echo “p,100” > /dev/[ARD_ADDR]` |
| Helium (Primary) Flow    | q | 0-10 (slm) |`$echo “q,1.5” > /dev/[ARD_ADDR]` |
| Oxygen (Secondary) Flow    | o   |   0-20 (sccm) | `$echo “o,1” > /dev/[ARD_ADDR]` |
| Frequency  | f|    10-20 (kHz) | `$echo “f,15” > /dev/[ARD_ADDR]` |
| Power   | w   |   1.5-5 (W) | `$echo “w,1.5” > /dev/[ARD_ADDR]` |
| X position | x   |   -50-50 (mm) | `$echo “x,1” > /dev/[ARD_ADDR]` |
| Y position | y   |   -50-50 (mm)| `$echo “y,1” > /dev/[ARD_ADDR]` |
| Z position | d   |    0-20 (mm) | `$echo “z,1” > /dev/[ARD_ADDR]` |
| Peak-to-Peak Voltage (for V12) | v | 0-10 (kV) | `$echo “v,8” > /dev/[ARD_ADDR]` |

**Notes** 
* By default the Z-position is set at 4mm - this is the nominal seperation distance
* Inputs are saturated at their maximum values, any value sent above the allowed range will result in the input to be set to its maximum value

## Serial Output
The firmware outputs a sequence of comma seperated values as output. The values correspond to the following variables:
``` 
Time stamp, p2p Voltage, Frequency, Helium Flow, Z position, Duty Cycle, Intensity 1, Intensity 2, Measured RMS Voltage, Embedded Temperature MEasurement, Measured RMS Current, X Position, Y Position, Oxygen Flow, Set Power, Measured Power
```
