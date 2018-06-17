
/*
PID Control with Thermal IR Input
Created and verified by Brandon Curtis (brandoncurtis.com) on 2015-11.

Temperature is read using a Melexis IR thermometer.
PID is used to control an output voltage via an MCP digital potentiometer.
In this implementation, potentiometer output was fed into a function generator to amplitude-modulate the output.
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <McpDigitalPot.h>
#include <PID_v1.h>

// Then choose any other free pin as the Slave Select (pin 10 if the default but doesn’t have to be)
#define MCP_DIGITAL_POT_SLAVE_SELECT_PIN 10

// It’s recommended to measure the rated end-end resistance (terminal A to terminal B)
float rAB_ohms = 10090.00; // 10090.00; 5k Ohm

// Instantiate McpDigitalPot object, with default rW (=117.5 ohm, its typical resistance)
McpDigitalPot digitalPot = McpDigitalPot( MCP_DIGITAL_POT_SLAVE_SELECT_PIN, rAB_ohms );

// Instantiate the thermopile
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// PID STUFF
// Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp = 1.0; // proportional control
double Ki = 1.0; // integral control
double Kd = 0.0; // derivative control

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // DIRECT or REVERSE linkage

void setup()
{
  Serial.begin(9600);
  // initialize SPI:
  SPI.begin();
  // Scale to 100.0 for a percentage, or 1.0 for a fraction
  digitalPot.scale = 100;
  // initialize thermopile
  mlx.begin();

  // PID STUFF
  Setpoint = 90;
  //turn the PID on
  myPID.SetTunings(Kp,Ki,Kd); // set the tuning parameters
  myPID.SetControllerDirection(DIRECT); // DIRECT or REVERSE
  myPID.SetOutputLimits(15,100);     	// potentiometer range is 0-100%
  myPID.SetSampleTime(50);          	// in milliseconds
  myPID.SetMode(AUTOMATIC);         	// AUTOMATIC = on
}

void loop()
{
  Input = mlx.readObjectTempF();
  if ( myPID.Compute() ) {
	digitalPot.setResistance(0, Output); // int value (0-100); PID is adjusting this!
  }
  Serial.println(Input);
}
