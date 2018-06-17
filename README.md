# Advanced Process Control for Atmospheric Pressure Plasmas

The work in this repository is done in the University of Californa, Berkeley by [**Graves Lab**](http://www.graveslab.org) and [**Mesbah Lab**](http://www.mesbahlab.com/)
*This repository is being regularly updated*

Atmospheric Pressure Plasma Jet (APPJ) devices exhibit highly variable behavior in response to changes in their environment. Small changes in, for example jet-tip-to-substrate sepration distane can cause drastic changes in the APPJ behavior. Using feedback control, we can mitigate some of this variability allowing for reliable, reproducible and effective operation of APPJs. Below, you can see how feedback control can help regulate substrate temperautre while the seperation distance changes (6x sped up). On the left you see the no control case, in the middle temperature is controlled by manipulating applied voltage and on the right, temperature is controlled manipulating the flow.

![Helium plasma jet](/Results/compare_pi.gif)

The maximum temperature measured at the surface for each case clearly demonstrates the need for and benefit of feedback control

![Helium plasma jet](/Results/Temp_prof.png)

For practial applications, the APPJ should also be translated over the substrate as often the APPJ sources have an effect area (~few mm^2) much smaller than practical targets (several cm^2).  Watch the plasma translating over a surface!

![Helium plasma jet](/Img/moving_jet.gif)

This repository contains code used for implementing classical (proportional-integral-derivative) and advanced (optimization-based) feedback control on a kHz-exctited atmospheric pressure plasma jet in helium 

## Hardware

## Firmware
The Arduino firmware allows exchanging commands with the Arduino via serial monitor. The latest version (as of 14/6/18) is version 14, available under /firmware/examples/dac/. The firmware is used for both the Master and Complementary Arduinos and is capable of managing two gas flow rates, 3D position, applied voltage and frequency. Default measurements include RMS voltage, RMS current, total emission intensity and ambient temperature.

## Software 
Detailed information about the software is under /experiments directory.Scripts are available for gathering detailed data from individual instruments under /experiments/scripts. The measure_server_PI and measure_server_OL scripts under /experiments/scripts allow for coordinated measurements from pheripherals and embedded systems simultaneously. These scripts also allow for interfacing with a websocket client to externally govern applied inputs or controller setpoints.



*This repository has been forked and modified from [**Plasma Analysis and Control**](https://github.com/brandoncurtis/plasma-control) by Brandon Curtis*
