

# Muting the Excitation Signal

## Principles



## Devices

### 2N5461

P-channel JFET

https://www.jameco.com/z/2N5461-Texas-Instruments-Transistor-JFET-P-Channel-TO-92-General-Purpose-Low-Level-Amplifier_253905.html
+ https://www.jameco.com/Jameco/Products/ProdDS/253905.pdf
+ https://www.onsemi.com/pub/Collateral/2N5460-D.PDF

Device features:
 • usually off; switched on by pulling the gate high

This device is usually off, and can be switched on by pulling the gate high.
This device is symmetrical; the source and drain are interchangeable.


### CD4016 and CD4066

Designed specifically for switching audio signals.

"Bidirectional Signal Transmission Via Digital Control Logic"

+ http://www.ti.com/lit/ds/symlink/cd4016b.pdf
+ http://www.ti.com/lit/ds/symlink/cd4066b.pdf

"The CD4066B device is a quad bilateral switch intended for the transmission or multiplexing of analog or digital signals. It is pin-for-pin compatible with the CD4016B device, but exhibits a much lower on-state resistance. In addition, the on-state resistance is relatively constant over the full signalinput range"


via: https://groupdiy.com/index.php?topic=27210.0

"The 4016's do not have buffers on the digital inputs so you can kinda 'soft switch' them a bit"

"The reason the parts are still desirable in some applications is the much lower transient induced into the channels of the switch FETs during switching.  The linearization parts in the 4066 have a nasty way of coupling something to one rail or the other during switching.  It can put a brutal transient in the audio, even when you switch in tens of nanoseconds.  If you are using the parts for pwm or switching modulators, etc., it renders them virtually unusable many times.

Otherwise, if you seldom switch, and if the impedances are reasonably low, the "on" switch variation in resistance versus voltage is much smaller in 4066, hence the distortion is also lower, making it the better part."


via: http://tech.thetonegod.com/switches/switches.html

"ICs: There are a number of ICs that can do analog audio signal switching. The most popular are the 4016, 4053, and 4066 which are part the digital logic family but are actually analog. They have excellent characteristics for audio signal switch. Good enough for our switching needs.

"The 4016 and 4066 are both bilateral switches. By bilateral we mean that signals can go in both directions through the switch. Both have four SPST switches. Both share the same pinouts therefore they can be used to replace each other. The difference is that the 4016 has a higher internal resistance across the switch. This means a loss of signal fidelity. The 4016 is really meant for other types of switching and is not optimized for audio switching. The 4066 is much better for audio switching.

"No matter which IC you choose to use you should use it at full supply voltage. The internal resistance of the switches increases when you use lower voltages. Audio problems can occur if you use them at the typical digital voltages of 3-5 volts. Since most effects use a 9v supply so use the full 9v. I have used them at as high as 15v with no problems."


via: http://cr4.globalspec.com/thread/67289/Difference-Between-4016-and-4066-ICs

"Main difference is that the 4066 has a much lower ON resistance than the 4016 (typ. 80Ω vs. 400Ω)."


### CD4066



### 2N7000



















## What I Actually Used

THIS has great designs, especially Figure 7 using the CD4066: http://sound.whsites.net/articles/muting.html

The topology I'm using is a version of "Figure 7 - CMOS Bilateral Switch Muting Circuit" from the second reference, powered by +/-9V split rails, and I'm using a (slightly less-clever?) design that utilizes two op amps configured as comparators (one inverting, one non-inverting) to sharpen the on/off transitions of the Arduino's PWM output and trigger the CD4066 switches.

+ https://www.jameco.com/z/CD4066-Major-Brands-QUAD-BILATERAL-SWITCH-DIP-14_13186.html
+ notebook: https://drive.google.com/open?id=0B1LciOIIcrN_ZmppNWF6bjVLQms
+ Audio switching notebook mentioned in the notes above: https://neatcircuits.com/audiosw/
+ 




