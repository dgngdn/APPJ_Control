Working With Arduino
====================

<!-- toc orderedList:0 depthFrom:2 depthTo:6 -->

* [References](#references)
* [Board Pinout](#board-pinout)
* [Working from the CLI](#working-from-the-cli)
* [Maximum Sample Rate](#maximum-sample-rate)
  * [ADC Rate Limitations](#adc-rate-limitations)
  * [See Also](#see-also)
* [Additional Hardware](#additional-hardware)
  * [DIY Arduino Shields](#diy-arduino-shields)
  * [Screw-Terminal Shield](#screw-terminal-shield)
* [Datalogging Examples](#datalogging-examples)

<!-- tocstop -->

## References

+ [BC Wiki: Arduino Hacking][bcwiki]

[bcwiki]: https://brandoncurtis.github.io/wiki/p/arduino

## Board Pinout

+ https://www.arduino.cc/en/reference/SPI
+ https://www.arduino.cc/en/reference/wire


## Working from the CLI

You can compile and upload code to an Arduino using the command-line interface:

    arduino --upload --port /dev/arduino --board arduino:avr:uno filename.ino







## Maximum Sample Rate

### ADC Rate Limitations

via: https://decibel.ni.com/content/message/60494


    You're going to have trouble hitting that rate with an Arduino.  The ADCs (analog to digital converters) on the ATMEL328 on the arduino uno have a theoretical maximum sample rate of something like 100KHz (I don't recall the exact rate, but lets just say: fast enough).

    So lets say we sample at 10 KHz with a 10 bit ADC.  This means we get 10,000 * 10 bits of data per second 100K/8 = 12.5KB (Kilo Bytes, not bits) per second.  The ATMEL 328 on an arduino uno has 2 KB of RAM which means we could fill the total ram on the microcontroller every .16 seconds.  The take away here is we cannot buffer much data on the Arduino.

    Now lets look at the interface.  The Arduino Uno does serial over USB using another ATMEL 328 (or an FTDI on older boards).  There is some buffering and other stuff that is going to slow down your throughput but lets say you run at a baud rate of 115200 (or 115200 bits per second).  At 10KHz and 10 bits per sample we would need to transfer 100,000 bits per second, which is just under that limit, however the baud rate of 115200 does guarantee a throughput of 115200 bits per second, you have to take into acount start and stop bits, bus collisions since we are using USB (a non deterministic bus), and much more.  In addition there are things on the Arduino that are going to take some amount of time, preventing us from transfering at our maximum rate.  For example the time it takes for the ADC to settle, the time to get the data from the ADC register to SRAM then from SRAM to the UART send buffer, etc.

    So now you start to see why proper DAQ devices, especially those with many channels, fast sample rates and high sample resolutions can get quite expensive.

    Now for some suggestions:

    The serial over USB and small amount of RAM are going to be limiting factors.  I recommend using something like the chipKIT Max32.  It has 128K of RAM (64 times more than the Arduino Uno).  This will allow you to buffer more samples on the device before sending chunks of data to LabVIEW.  The Max32 also gives you the option of using 'USB for Serial' (it's in the board type selection box in MPIDE).  This allows you to use the Arduino style serial API, but instead of actually transfering serial data to another chip and then that chip converting the data to USB packets and sending them, it uses the USB device controller on the PIC32 microcontroller that all of your code is running on.  This allows you to get much faster speeds because you are no longer actually sending RS232 style serial data and can take advantage of the extra speed USB provides.

    As for the code, I would try to use interrupts to trigger an A to D conversion at regular intervals (say .1mS if you want 10KHz).  Store each sample in a circular buffer (wikipedia if you don't know what this is).  The buffer will need a pointer to the next sample to send and the next slot to store data in.  When you're program is not interrupted for a sample it should be piping as much data out over USB as possible but try to do it in large chunks rather than a byte at time.  USB sends data in packets and a byte a time will result in many packets even if the packet payload could contain many more bytes.

    It might be a good idea to lower the sample resolution to 8 bits so you don't have to mess with byte packing.  For example if you have a 10 bit sample you have to send 2 full bytes, or pack data together to send the 8 LSb of the first sample, then the second byte with the 2 MSb of the first sample the the 6 LSb of the next sample.  This packing takes time on the uController but saves time on the transfer.  You'll have to play around with this and see what rates you're actually able to hit.

    On the LabVIEW side you'll get an array of numbers.  You can easily build these into a wave form by adding a T0 and DT (there is a build waveform vi, I don't recall exactly what it's called) and then display it on a waveform graph.

    If you want to get fancy you could also have an interrupt for receive data that allows you to send a packet with a new DT (time between samples).  This would allow you to change the sample rate on the fly.

    If you're project is to build a DAQ device you can certainly make a simple one using an Arduino (or better yet a chipKIT).  However if you're project is to build some other system and the DAQ piece is just a small component of the larger system it is well worth buying a cheap DAQ device to save yourself the time it will take to build it from scratch.

    Sorry for the massive post, but I'm going off the grid for a few days so it had to be one massive brain dump.  Good luck, let us know how it goes.

    -Sam K
    LIFA Developer
    www.labviewhacker.com
    Google Plus


### See Also

Using the approach given this web page you can get 50 KHz.
<https://www.inkling.com/read/arduino-cookbook-michael-margolis-2nd/chapter-18/recipe-18-9>

This web page says 100 KHz when logging to a SD card.
<http://forums.adafruit.com/viewtopic.php?f=31&t=30557>

This web page says 5 million samples per second when using an external ADC.
<http://hackaday.com/2013/07/08/arduino-oscilloscope-at-five-megasamples-per-second/>

For detailed information from Atmel see.
<http://www.atmel.com/dyn/resources/prod_documents/DOC2559.pdf>


## Additional Hardware

### DIY Arduino Shields

Screw Terminal, 2.54mm pitch
10x1, 8x2, 6x1

0.1″ (2.54 mm) Screw Terminal Blocks
https://www.pololu.com/category/177/0.1-2.54-mm-screw-terminal-blocks

Electronics-Salon 10x Prototype PCB for Arduino UNO R3 Shield Board DIY
https://www.amazon.com/dp/B01J1KM3RM
$13.50


### Screw-Terminal Shield

Proto-Screwshield (Wingshield) R3 Kit for Arduino
https://www.adafruit.com/product/196
$15

ProtoScrewShield Kit
https://www.sparkfun.com/products/9729
$15

DFRobot Screw Shield, Wings
https://www.jameco.com/z/DFR0060-DFRobot-Screw-Shield-Converts-Header-Pins-To-Screw-Terminals_2144534.html
$8

DFRobot Screw Shield
https://www.jameco.com/z/DFR0131-DFRobot-Proto-Screw-Shield-Assembled-Arduino-Compatible-_2159314.html
$16

Boardproto Screw Shield
https://www.itead.cc/itead-proto-screw-shield.html
"Missing the I2C and the two ICSP header pins for the UNO rev 3"
$3.50

Arduino Proto Screw Shield
https://www.amazon.com/iTead-IM120417013-Arduino-Proto-Shield/dp/B00HBVVKPA
"Missing the I2C and the two ICSP header pins for the UNO rev 3"
$7

Electronics-Salon Prototype Screw Shield Board Kit For Arduino UNO R3, 0.1" Mini Terminal Block.
https://www.amazon.com/Electronics-Salon-Prototype-Shield-Arduino-Terminal/dp/B00UT0QLJA/
"Kit"
$11

Gikfun Screw Shield Expansion Board For Arduino UNO R3 EK7007
https://www.amazon.com/Gikfun-Shield-Expansion-Arduino-EK7007/dp/B014SGTP20/
"screw terminals face inwards"
$10

Screw Shield, Wings
https://www.amazon.com/LinkSprite-101101019-Screw-Shield/dp/B018260AAI/
$10


## Datalogging Examples

+ https://github.com/adafruit/Light-and-Temp-logger
+ https://learn.adafruit.com/adafruit-data-logger-shield/using-the-real-time-clock-2
+ https://www.adafruit.com/product/201
+ https://www.sparkfun.com/products/11166
+ https://www.arduino.cc/en/Guide/ArduinoEthernetShield#toc3
+ https://www.arduino.cc/en/Reference/SD
+ https://www.arduino.cc/en/Tutorial/Datalogger
+ https://www.arduino.cc/en/Tutorial/ReadWrite
+ https://www.arduino.cc/en/Reference/SDCardNotes
+ https://www.arduino.cc/en/Tutorial/CardInfo
+ https://www.arduino.cc/en/Tutorial/DumpFile
+ https://www.arduino.cc/en/Tutorial/Files
+ https://www.arduino.cc/en/Tutorial/Listfiles
