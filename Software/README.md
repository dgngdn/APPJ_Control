## Manual Measurements
Manual measurement scripts are available for pheripheral instruments. They are `oscilloscpoe.py`, `thermography.py` and `spectroscopy.py`. By default these scripts save the raw data from the corresponding instrument under /data in the current directory. The following options are available for these scripts

**Common Options**
* `--dir` - allows manually selecting the directory in which the files are saved
* `--loop` - allows for continiously logging data by looping the script

**Script-specific Options**

**`oscilloscope.py`**
* `--chan` - selects the channel(s) from which data is acured *this options is required!*
* `--platform` - allows for selecting between `usbtmc` and `visa` communication protocols
* `--timeout` - allows manually selecting measurement timeout - 0.4s by default

**`spectroscopy.py`**
* `--integrate` - selects the OES integration time in microseconds - 200,000 us by default

## Automated Experiments
`measure_server_PI.py` and `measure_server_OL.py` scripts allow for automated experiments either through hard coding the desired conditions in the script or via `socket_client` scripts running on a seperate device for coordination. `OL_experiment.py` is a depriciated method for testing desired input sequences in an automated manner. Following options are available for these scripts:

**`measure_server_PI.py/measure_server_OL.py`**
* `--dir` - allows manually selecting the directory in which the files are saved
* `--tag` - allows for adding a tag to the output file for easy recoginition - by default the file names are timestaps for experiment start time
* `--auto` - runs the code autonomously, without expecting a connection from an websocket external client
* `--save_therm` - saves the raw thermal camera images at each measurement time
* `--save_spec` - saves the raw optical emission spectrum data at each measurement time 

**`OL_Experiment.py`(Depreciated)**
This script reads input values from a `.mat` file hardcoded into the script in the form [V,f,q,d] in terms of difference from a nominal operating point [8,12,1.2,4]. The outputs are saved in the local directory with timestamp.


## Output 
The scripts **`measure_server_PI.py/measure_server_OL.py`** output a timestamped file of comma seperated values that correspond to following variables
```
Time,Temperature Setpoint,Maximum Surface T,Ts2,Ts3,Power,Maximum Current,OES intensity at O777, OES at O845,OES at N391,OES at He706,Total OES Intensity,p2p Voltage, Frequency, He Flow , position,X position,Y position,Embedded Temperature, Power Setpoint,Power measurement,Power measurement from oscilloscope,Duty Cycle, Measurement Sampling time
```

**Note**:`Ts2` and `Ts3` are spatially resolved temperature measurements used to characterize the temperature distribution. `Ts2` is 2 pixels off the maximum value and `Ts3` is 12 pixels away. Where these measurements are taken need to be calibrated based on the distance between the substrate and the thermal camera.

## Socket Clients
Socket clients allow for interacting with the `measure_server_PI.py/measure_server_OL.py` scripts via an external computer over WiFi. The script `socket_client_feed_setpoint.py` can be used to feed the `measure_server` scripts a series of setpoints or input values (some modification of code is needed). On the other hand `socket_client_feed_setpoint.py` runs an optimization based supervisory control algorithm using [CasADi](http://web.casadi.org/) to achieve 1D dose delivery.
