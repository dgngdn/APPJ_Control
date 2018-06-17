


## Prefabricated Unix utility

There is a very nice Unix utility that will take care of this for you:

    sudo apt install moreutils
    cat /dev/arduino | ts "%Y-%M-%d_%H:%M:%.S"


### Prepending system time to a stream written to a file with shell scripts

THIS WORKS, BUT STILL HAS BUFFERING PROBLEMS:
unbuffer cat /dev/arduino | awk '{ "date +%Y-%m-%d_%T.%3N" | getline timestamp; close ("date +%Y-%m-%d_%T.%3N"); print timestamp,","$0; fflush(); }' >> temperaturehistory


references:
https://www.gnu.org/software/gawk/manual/html_node/Getline_002fVariable_002fPipe.html
https://www.gnu.org/software/gawk/manual/html_node/Getline.html
https://www.gnu.org/software/gawk/manual/html_node/Close-Files-And-Pipes.html#Close-Files-And-Pipes

cat /dev/ttyACM0 | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
This version prepends the script runtime, forever and ever, to the incoming text.

cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
This one updates the time but appears to insert itself rather randomly
into the incoming bitstream, garbling it.
Nevermind! after refreshing the serial port with whatever settings the Arduino IDE applies,
it looks great now.
Does strftime offer milliseconds or better? no.
http://manpages.ubuntu.com/manpages/xenial/man3/strftime.3.html
so, I should get this exampel working with `date` built-in.

cat /dev/ttyACM0 | sed 's/\r\n/$(date +"%Y-%m-%d %H:%M:%S")\r\n/g' >> com1.txt
This does not appear to contain \r\n\ line breaks, so it fails to insert the date!
...on further inspection, it DOES contain those \r\n line breaks! not sure what's up...
Using this to write to a file is" also delayed.
\r = carriage return (return to beginning of line)
\n = line feed or newline, move down one line to keep typing

Let's do some additional escaping on that one, and switch to double quotes
to allow shell expansion:
cat /dev/ttyS0 | sed "s/\r\n/$(date \+\"%Y-%m-%d\ %H:%M:%S\")\r\n/g"
NOPE

cat /dev/ttyACM0 | awk '{print "date +%Y-%M-%d_%H:%M:%S:%N", $0; }'

http://stackoverflow.com/questions/21564/is-there-a-unix-utility-to-prepend-timestamps-to-stdin

@teh_senaus As far as I know, awk's strftime() does not have millisecond precision. However you can use the date command. Basically you just trim nanoseconds to three characters. It will look like this: COMMAND | while read -r line; do echo "$(date '+%Y-%m-%d %T.%3N') $line"; done. Note that you can abbreviate %H:%M:%S with %T. If you still want to use awk for some reason, you can do the following: COMMAND | awk '{ "date +%Y-%m-%d\\ %T.%3N" | getline timestamp; print timestamp, $0; fflush(); }

My command history:

 1629  mount -v | grep "^/" | awk '{print "\nPartition identifier: " $1  "\n Mountpoint: "  $3}'
 2004  cat /dev/ttyACM0 | awk '{print $1)'
 2005  cat /dev/ttyACM0 | awk '{print $1}'
 2006  cat /dev/ttyACM0 | awk '{print $1 $1 $1}'
 2007  cat /dev/ttyACM0 | awk '{print $1 $0 $1 $2}'
 2008  echo "foo,bar" | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2009  cat /dev/ttyACM0 | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2010  tail -f /tmp/asdf | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2011  cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
 2013  cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
 2030  cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
 2032  cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
 2035  cat /dev/ttyACM0 | awk '{print strftime("%Y-%m-%d %H:%M:%S %z"), $0; }'
 2036  cat /dev/ttyACM0 | awk '{print date +%Y-%M-%d_%H:%M:%S:%N, $0; }'
 2037  cat /dev/ttyACM0 | awk '{print date +"%Y-%M-%d_%H:%M:%S:%N", $0; }'
 2038  awk '{print date +"%Y-%M-%d_%H:%M:%S:%N"}'
 2039  awk '{print `date +"%Y-%M-%d_%H:%M:%S:%N"`}'
 2040  awk '{print $(date +"%Y-%M-%d_%H:%M:%S:%N")}'
 2043  history | grep awk

 2001  echo "foo,bar" | awk '{x="'"`date +%Y%M%d%S%N`"'"; printf "%s,%s\n",x,$0 }'
 2002  echo "foo,bar" | awk '{x="'"`date +%Y-%M-%d_%S%N`"'"; printf "%s,%s\n",x,$0 }'
 2003  echo "foo,bar" | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2004  echo "foo,bar" | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S.%N/1000000000`"'"; printf "%s,%s\n",x,$0 }'
 2005  echo "foo,bar" | awk '{x="'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2013  tail -f tmp/asdf | awk '{"'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }
 2014  tail -f tmp/asdf | awk '{"'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2015  tail -f /tmp/asdf | awk '{"'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s,%s\n",x,$0 }'
 2016  tail -f /tmp/asdf | awk '{"'"`date +%Y-%M-%d_%H:%M:%S:%N`"'"; printf "%s\n",x,$0 }'

cat /dev/ttyACM0 | awk '{print $(date +%Y-%M-%d_%H:%M:%S:%N), $0; }'

annnnnnnnnnnnnd HERE WE GO:
tail -f /tmp/asdf | while read -r line; do echo "$(date '+%Y-%m-%d %T.%3N') $line"; done

USE THIS AND DITCH THE STUPID RTC.

### stty EOL and hangup settings


       * dsusp CHAR
              CHAR will send a terminal stop signal once input flushed

       eof CHAR
              CHAR will send an end of file (terminate the input)
       intr CHAR
              CHAR will send an interrupt signal

       quit CHAR
              CHAR will send a quit signal
       stop CHAR
              CHAR will stop the output

       susp CHAR
              CHAR will send a terminal stop signal


X UPDATE THERMOGRAPHY SKETCH
X REMOVE RTC → V11
X CLEAN UP ANALYZE / LAUNCH
X COME UP WITH MONITORING TIMER SOLUTION
X INVESTIGATE ARDUINO I2C PULL-UP RESISTOR SITUATION
--------------
2016-05-11
removing excess from launch.ipynb... DONE
adding script to reset the camera... DONE
--------------
realtime ADC monitoring of current
embedded thermocouple + amp
thin conductive plate
fiber optics








------------
### Controlling when the serial port hangs up

????

### Control which serial port a device connects to:
http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/



### before and after the Arduino IDE fixes the serial output

brandon@D105:~$ stty -F /dev/ttyACM0 --a
speed 9600 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^A; eol = <undef>; eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R; werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 0;
-parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts
-ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke

brandon@D105:~$ stty -F /dev/ttyACM0 --a
speed 9600 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^A; eol = <undef>; eol2 = <undef>; swtch = <undef>; start = ^Q; stop = ^S; susp = ^Z; rprnt = ^R; werase = ^W; lnext = ^V; flush = ^O; min = 0; time = 0;
-parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts
-ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke

now cat doesn't work...

and now after raw, when cat works great:

brandon@D105:~$ stty -a < /dev/ttyACM0
speed 9600 baud; rows 0; columns 0; line = 0;
intr = ^C; quit = ^\; erase = ^?; kill = ^U; eof = ^A; eol = <undef>; eol2 = <undef>; swtch = <undef>; start = ^Q;
stop = ^S; susp = ^Z; rprnt = ^R; werase = ^W; lnext = ^V; flush = ^O; min = 1; time = 0;
-parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts
-ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8
-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0
-isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke


http://www.computerhope.com/unix/ustty.htm
-inpck → inpck		"enable input parity checking"
min = 1 → min = 0	"with -icanon, set N characters minimum for a completed read"

"icanon - enable erase, kill, werase, and rprnt special characters"

=======================
how to tell what is currently connected to the serial port
=======================
this person states:
http://playground.arduino.cc/Interfacing/LinuxTTY

Specifically: The arduino-0015 development application does not close or release /dev/ttyUSB0 on Linux systems completely.

As a result, "cat /dev/ttyUSB0" fails if you have used the serial data display in the Arduino development application.

To re-capture the data use "screen"

screen /dev/ttyUSB0     // Kill screen with ^ak or control-a k

An alternate way to stop the screen program: Display the process table and kill screen using the PID in column two.

ps aux         // see screen and SCREEN processes. Note PID in column 2
kill 9264 9265 // screen is gone and "cat /dev/ttyUSB0" now works
=======================

### Power cycle the FLIR Lepton on i2c from the RPi GPIO

----

## RTC Hardware


Using a $2 DS3231 RTC & AT24C32 EEprom from eBay
https://edwardmallon.wordpress.com/2014/05/21/using-a-cheap-3-ds3231-rtc-at24c32-eeprom-from-ebay/
This guy is building data loggers to leave in caves!
HUGE resource on these cheap RTCs and low-power logger design!

Info on this device, including how to change the i2c bus address:
http://www.dx.com/p/ds3231-high-precision-real-time-clock-module-blue-3-3-5-5v-222910#.VzVVrrorKkA
https://www.maximintegrated.com/en/products/digital/real-time-clocks/DS3231.html
