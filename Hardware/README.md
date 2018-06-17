## Instrumentation 

### List of Parts
#### Plasma Source
* Quartz tube ID=3mm, OD=4mm
* Copper sheet 10mm wide wrapped around the tube 1cm from the tip
### Actuation
* Arduino UNO x 2 for actuation and embedded measurement
* Raspberry Pi 3 for coordinating measurements and manage data aquisition from different sources
* Function generator
  * MCP4922 - 12 bit Digital to Analog Converter
  * MC34072P - Dual Op-Amp
  * XR2206CP - Monolithic Function Generator
  * CD4016BE - Analog Switch
* Trek 10kV/40mA HV Amplifier
* Adafruit Motor Shield x 2 (or equivalent stepper motor driver)
* Nema 19 Stepper Motor x 3
* X-Y Movement table
* Lead screw and slider assembly for Z-axis actuation
### Embedded Measurements
* AD536A - AC to RMS converter for voltage and current measurement
* Photodarlington diode for intensity measurement 
* MLX90614 (Optional) for point measurement of temperature
### Peripherals
* Oscilloscope RIGOL DS1000D (Only 2 channels used)
* Optical Emission Spectrometer - Oceanoptics 
* FLIR Lepton 2.5 Radiometric Thermal Camera


## Layout

The control system consists of embedded measurements and actuation managed by two Arduino UNO controllers. A Raspberry Pi 3 is used to coordinate the microntrollers and pheripheral instruments. The Master Arduino Manages actuation of applied voltage, frequency, duty cycle and gas flow rates via homebrewed circutiry. (See \hardware section for details on instruments and ciruit diagrams). The Master Arduino also measures RMS applied voltage and RMS current via AC-to-RMS converter circuits as well as total light intensity via a photodarlington. The x,y and z position of the jet is coordinate by the Complementry Arduino equipped with two Adafruit Motor Control Shields. See the diagram below for information flow relating to the Arduino UNOs. 

![Helium plasma jet](/Img/APPJ_diagram_embedded.png)

As pheripherals, a FLIR lepton 2.5 radiometric thermal camera, RIGOL D1000Z oscilloscope and Oceanoptics optical emission spectrometer are configured. Information flow relating to the pheripheral instruments are shown below.

![Helium plasma jet](/Img/APPJ_diagram_pheripherals.png)

