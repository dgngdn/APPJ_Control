/* http://arduino.cc/en/Hacking/LibraryTutorial */
/* http://ww1.microchip.com/downloads/en/DeviceDoc/22250A.pdf */
// Brandon: THIS WORKS 2017-04-19

// DEFINES
#define DEBUG false

// IMPORTS
#include <MCP4922.h>
#include <SPI.h>

const int PIN_DAC_A = 2;
const int PIN_DAC_B = 3;
const int DAC_STEPS = 4096;
int step_interval = 10;

//MCP4922 DAC(51,52,53,5);    // (MOSI,SCK,CS,LDAC) define Connections for MEGA_board, 
MCP4922 DAC(11,13,10,5);    // (MOSI,SCK,CS,LDAC) define Connections for UNO_board, 

namespace data {
  int DAC_A;
  int DAC_B;
  int DAC_SUM;
}

namespace setpoint {
  float voltage;
  float frequency;
  float flowrate;
}

float mapfloat(float x, long in_min, long in_max, long out_min, long out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void get_serial() {
  // gets manual input from the serial port
  #if DEBUG
    Serial.println("getting serial input...");
  #endif

  String inputString = "";         // a string to hold incoming data
  boolean stringComplete = false;  // whether the string is complete

  while (Serial.available() > 0) {

    // get the new byte:
    char inChar = (char)Serial.read();
    
    #if DEBUG
      Serial.println(String(inChar));
    #endif
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      break;
    }
  }

  if ( stringComplete == true && inputString.length() > 0 ) {
    // if you have a non-blank input string and it's complete...
    #if DEBUG
      Serial.print("got serial input: ");
      Serial.print(inputString);
      Serial.println();
    #endif
    // process it with your manual_input function!
    manual_input(inputString);
  }

  //return inputString.toInt();
}

void manual_input(String input) {
  // process the manual request recieved via serial
  #if DEBUG
    Serial.println("processing serial input!");
  #endif

  switch( input.charAt(0) ) {
    case 'v' :
      // you sent v,###
      setpoint::voltage = input.substring(2).toFloat();
      #if DEBUG
        Serial.println("voltage set!");
      #endif
      actuate_inputs();
      break;
  }
}

void actuate_inputs() {
  // actuates the system inputs via the DACs
  #if DEBUG
    Serial.println("actuating inputs...");
  #endif
  DAC.Set(mapfloat(setpoint::voltage,0,10,4095,0), 0);
}







void setup()
{
  Serial.begin(9600);
  SPI.begin();
}

void loop()
{
  if (Serial.available() > 0) {
    #if DEBUG
      Serial.println("bytes available: " + String(Serial.available()));
    #endif
    get_serial();
  }
  
  data::DAC_A = analogRead(PIN_DAC_A); // analogRead(A0); //analogRead(PIN_DAC_A);
  data::DAC_B = analogRead(PIN_DAC_B); // analogRead(A1); //analogRead(PIN_DAC_B);
  data::DAC_SUM = data::DAC_A + data::DAC_B;
  Serial.print(data::DAC_A);
  Serial.print(",");
  Serial.print(data::DAC_B);
  //Serial.print(",");
  //Serial.print(data::DAC_SUM);
  Serial.println();
  delay(50);
}

