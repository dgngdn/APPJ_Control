#include <MCP4922.h>

char junk;
/* http://arduino.cc/en/Hacking/LibraryTutorial */
/* http://ww1.microchip.com/downloads/en/DeviceDoc/22250A.pdf */
// Brandon: TESTING 2017-04-19

// running with 24V power supply for op amp and function generator

/* This minimal example shows how to get single-shot range
measurements from the VL6180X.
The range readings are in units of mm. */

// DEFINES
#define DEBUG false

// IMPORTS
#include <Wire.h>
#include <VL6180X.h>  // library: VL6180X proximity sensor
#include <avr/wdt.h>  // library: watchdog timer
#include <MCP4922.h>  // library: DAC
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MLX90614.h>  // library: MLX90614 thermopile
#include <SPI.h>
#include <Wire.h>

// GLOBAL VARS
VL6180X proxsensor;
//MCP4922 DAC(51,52,53,5);    // (MOSI,SCK,CS,LDAC) define Connections for MEGA_board
MCP4922 DAC_FXN(11,13,10,5);  // (MOSI,SCK,CS,LDAC) define Connections for UNO_board
MCP4922 DAC_MFC(11,13,9,5);   // (MOSI,SCK,CS,LDAC) define Connections for UNO_board
Adafruit_MLX90614 mlx_5deg = Adafruit_MLX90614(0x5A);  // default address of MLX90614 thermopile

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS(0x60); //top shield
Adafruit_MotorShield AFMS_bot(0x61); //bottom shield

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M3 and M4)
Adafruit_StepperMotor *dMotor = AFMS.getStepper(200, 1);
// to motor port #1 (M1 and M2)
Adafruit_StepperMotor *xMotor = AFMS.getStepper(200, 2);
// to motor port #1 (M1 and M2) bottom shield
Adafruit_StepperMotor *yMotor = AFMS_bot.getStepper(200, 1);

String inputString = "";         // a string to hold incoming data

const int read_num = 100;
int read_data[read_num];
float read_average = 0;
int read_index = 0;
int read_total = 0;

// GLOBAL CONSTANTS
const double SERIAL_BAUD = 38400;
const int PROXIMITY_TIMEOUT = 25;
const int LOOPDELAY = 1; // milliseconds
const int DACSTEPS = 4096;
const int t_samp = 20; //milliseconds
double t_prev = 0;

// PINOUT
const int PIN_PHOTO_A = 0;
const int PIN_PHOTO_B = 1;
const int PIN_DAC_A = 2; 
const int PIN_DAC_B = 3;
const int PIN_V_RMS = A3;
const int PIN_I_RMS = A2;
const int PIN_PWM = 5;

namespace data {
  int dac_a;
  int dac_b;
  int dist;
  int x_pos;
  int photo_a;
  int photo_b;
  float v_rms;
  float i_rms;
  float t_emb;
}

namespace  PI_V{
  float Kc = 10/(0.71*30);
  float Tau_i = 10;
  float I = 0;
  float err = 0 ;
}

namespace setpoint {
  float voltage = 0;
  float vapp = 0;
  float frequency = 10;
  float flowrate = 0;
  float dist = 40;
  float x_pos = 0;
  float y_pos = 0;
  float duty = 0;
}

namespace location {
  float delta = 0;
  float cur_loc = 40;
  float delta_x = 0;
  float cur_x = 0;
  float delta_y = 0;
  float cur_y = 0;
}

static const uint8_t PROGMEM dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

uint8_t crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  while (len--) {
    crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
  }
  return crc;
}






void setup_serial() {
  // set up serial connection
  // initialize serial communication at specified baudrate
  Serial.begin(SERIAL_BAUD);
  #if DEBUG
    Serial.println();
    Serial.println();
    Serial.println("Serial connection established!");
  #endif
}

void setup_watchdog() {
  // enable the watchdog timer
  // if Arduino stops responding, auto-resets
  // Watchdog Timeouts: WDTO_{1,2,4,8}s
  // WDTO_{15,30,60,120,250,500}MS
  // http://www.megunolink.com/articles/how-to-detect-lockups-using-the-arduino-watchdog/
  #if DEBUG
    Serial.println("enabling watchdog timer...");
  #endif
  wdt_enable(WDTO_8S);
  
  #if DEBUG
    Serial.println("watchdog timer enabled!");
  #endif
}

////////////////////// READING FROM SERIAL ////////////////////////
////// GET STRING ////////////////////////////////////////////////

void get_serial() {
  // gets manual input from the serial port
  #if DEBUG
    Serial.println("getting serial input...");
  #endif

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
    inputString = "";
  }
}

////// PARSE INPUTS /////////////////////////////////////////////////////

void manual_input(String input) {
  // process the manual request recieved via serial
  #if DEBUG
    Serial.println("processing serial input!");
  #endif

  switch( input.charAt(0) ) {
    case 'v' :
      // you sent v,###
      if (input.substring(2).toFloat() > 10) {setpoint::voltage=10;}
      else { 
      setpoint::voltage = input.substring(2).toFloat();}
      #if DEBUG
        Serial.println("voltage set!");
      #endif
      actuate_inputs();
      break;
  
    case 'f' :
      // you sent f,###
      if (input.substring(2).toFloat() > 20) {setpoint::frequency=20;}
      else if (input.substring(2).toFloat() < 10) {setpoint::frequency=10;}
      else { 
      setpoint::frequency = input.substring(2).toFloat();};
      #if DEBUG
        Serial.println("frequency set!");
      #endif
      actuate_inputs();
      break;
      
    case 'q' :
      // you sent q,###
      if (input.substring(2).toFloat() > 10) {setpoint::flowrate=10;}
      else if (input.substring(2).toFloat() < 0) {setpoint::flowrate=0;}
      else { 
      setpoint::flowrate = input.substring(2).toFloat();};
      #if DEBUG
        Serial.println("flowrate set!");wa
      #endif
      actuate_inputs();
      break;
      
      case 'd' :
      // you sent d,###
      if (input.substring(2).toFloat() > 15) {setpoint::dist=150;}
      else if (input.substring(2).toFloat() < 0) {setpoint::dist=0;}
      else {
      setpoint::dist = input.substring(2).toFloat()*10;}
      location::delta = -location::cur_loc + setpoint::dist;
      //Serial.println(location::delta);     
      #if DEBUG
        Serial.println("stepper distance set!");
      #endif
      actuate_inputs();
      break;

     case 'x' :
      // you sent x,###
      if (input.substring(2).toFloat() > 1000) {setpoint::x_pos=1000;}
      else if (input.substring(2).toFloat() < -1000) {setpoint::x_pos=-1000;}
      else {
      setpoint::x_pos = input.substring(2).toFloat();}
      location::delta_x = -location::cur_x + setpoint::x_pos;
      //Serial.println(location::delta);     
      #if DEBUG
        Serial.println("stepper distance set!");
      #endif
      actuate_inputs();
      break;

       case 'y' :
      // you sent x,###
      if (input.substring(2).toFloat() > 1000) {setpoint::y_pos=1000;}
      else if (input.substring(2).toFloat() < -1000) {setpoint::y_pos=-1000;}
      else {
      setpoint::y_pos = input.substring(2).toFloat();}
      location::delta_y = -location::cur_y + setpoint::y_pos;
      //Serial.println(location::delta);     
      #if DEBUG
        Serial.println("stepper distance set!");
      #endif
      actuate_inputs();
      break;
      
      case 'p' :
      // you sent p,###
      setpoint::duty = input.substring(2).toFloat();
      #if DEBUG
        Serial.println("duty cycle set!");
      #endif
      actuate_inputs();
      break;
  }
}

////////////////////////////// STEPPER MOVEMENT //////////////////////////////////


int move_to_pos(float delt, Adafruit_StepperMotor *motor) {
  //Serial.print(aaReal.x);
  //Serial.print('\n');

  if (delt < 0) {
      motor->step(10, FORWARD, DOUBLE); //// STEP SIZE calibrated 12/08/2017
      motor->release();
      delt=delt + 1;
      delay(5);
     return delt;
  } else if (delt > 0) {
      motor->step(10, BACKWARD, DOUBLE); 
      motor->release();
      delt=delt - 1;
      delay(5);
    return delt;
  } else if (delt == 0) {
    return delt;
  }
}

int move_to_pos_x(float delt, Adafruit_StepperMotor *motor) {
  //Serial.print(aaReal.x);
  //Serial.print('\n');

  if (delt < 0) {
      motor->step(10, FORWARD, MICROSTEP); //// STEP SIZE calibrated 02/07/2018
      //motor->release();
      delt=delt + 1;
      delay(2);
     return delt;
  } else if (delt > 0) {
      motor->step(10, BACKWARD, MICROSTEP); //// STEP SIZE calibrated 02/07/2018
      //motor->release();
      delt=delt - 1;
      delay(2);
    return delt;
  } else if (delt == 0) {
    return delt;
  }
}
////////////////////////////////////// ACTUATION ///////////////////////////////////////

void actuate_inputs() {
  // actuates the system inputs via the DACs
  #if DEBUG
    Serial.println("actuating inputs...");
  #endif
  //DAC_FXN.Set(mapfloat(setpoint::voltage,0,10,0,DACSTEPS-1), mapfloat(setpoint::frequency,0,10,DACSTEPS-1,0));
  //DAC_FXN.Set(mapfloat(setpoint::voltage,0,10,((float) 6 / 10)*DACSTEPS,((float) 2 / 10)*DACSTEPS), mapfloat(setpoint::frequency,0,10,DACSTEPS-1,0));
  
  // tuned for Brandon's development setup on 24V supply:
  //DAC_FXN.Set(mapfloat(setpoint::voltage,0,10,0.96*(DACSTEPS-1),0.55*(DACSTEPS-1)), 
  //        mapfloat(setpoint::frequency,10,20,0.92*(DACSTEPS-1),0.1*(DACSTEPS-1)));

  // tuned for the control jet setup on 24V supply:

  ////////////// CONTROL FOR VGAP //////////////////////////////////////////////////
  setpoint::vapp=setpoint::vapp + PI_V::Kc*(PI_V::err+PI_V::I/PI_V::Tau_i);  
  
  if (setpoint::vapp > 10) {setpoint::vapp=10;} /// SATURATION
  else if (setpoint::vapp < 0) {setpoint::vapp=0;};

  // tuned for the control jet setup on 24V supply:
  DAC_FXN.Set(mapfloat(setpoint::vapp,0,12,0.96*(DACSTEPS-1),0.46*(DACSTEPS-1)), 
          mapfloat(setpoint::frequency,10,20,0.90*(DACSTEPS-1),0.085*(DACSTEPS-1)));

//voltage control off
 //DAC_FXN.Set(mapfloat(setpoint::voltage,0,12,0.96*(DACSTEPS-1),0.46*(DACSTEPS-1)), 
 //      mapfloat(setpoint::frequency,10,20,0.90*(DACSTEPS-1),0.085*(DACSTEPS-1)));
  
  // MFC DAC
  DAC_MFC.Set(mapfloat(setpoint::flowrate,0,10,0,4095),0);

 location::delta=move_to_pos(location::delta, dMotor);
 location::cur_loc = -location::delta + setpoint::dist;

 location::delta_x=move_to_pos_x(location::delta_x, xMotor);
 location::cur_x = - location::delta_x + setpoint::x_pos;

 
 location::delta_y=move_to_pos_x(location::delta_y, yMotor);
 location::cur_y = - location::delta_y + setpoint::y_pos;
 
   // set duty cycle
  analogWrite(PIN_PWM, mapfloat(setpoint::duty,0,100,0,255));
 
}

float mapfloat(float x, long in_min, long in_max, long out_min, long out_max)
{
 return (x - in_min) * ((float)out_max - out_min) / ((float)in_max - in_min) + out_min;
}

void addRead(int value)
{
  read_total = read_total - read_data[read_index];
  read_data[read_index] = value;
  read_total = read_total + read_data[read_index];
  read_average = ((float)read_total) / read_num;
  read_index++;
  if (read_index >= read_num)
  {
    read_index = 0;
  }
}

/////////////////////////// SETUP /////////////////////////////

void setup()
{
  setup_serial();
  setup_watchdog();
  SPI.begin();
  Wire.begin();
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  AFMS_bot.begin();  // create with the default frequency 1.6KHz
  
  dMotor->setSpeed(10);  // 10 rpm   
  xMotor->setSpeed(30);  // 10 rpm   
  yMotor->setSpeed(30);  // 10 rpm   

  // set up VL6180X sensor
  proxsensor.init();
  proxsensor.configureDefault();
  proxsensor.setTimeout(PROXIMITY_TIMEOUT);

  actuate_inputs();
}

/////////////////////////// LOOP ////////////////////////////

void loop()
{
  // reset the watchdog timer
  // if this doesn't occur within WDTO_X, system resets
  wdt_reset();
  
  if (Serial.available() > 0) {
    #if DEBUG
      Serial.println("bytes available: " + String(Serial.available()));
    #endif
    get_serial();
  }

  // get the timestamp
  double ts = millis();
  
  // read and save data from sensors
  data::dac_a = analogRead(PIN_DAC_A);
  data::dac_b = analogRead(PIN_DAC_B);
  data::photo_a = analogRead(PIN_PHOTO_A);
  data::photo_b = analogRead(PIN_PHOTO_B);
  data::v_rms = ((analogRead(PIN_V_RMS)*5.0/1024.0) -0.0029)/0.97;
  data::i_rms = (analogRead(PIN_I_RMS)*5.0/1024.0);
  data::dist = proxsensor.readRangeSingleMillimeters(); // read VL6180X distance
  data::t_emb = mlx_5deg.readObjectTempC();
  //addRead(data::dist); // add distance to averaging array

    PI_V::err = 0.71*(setpoint::voltage/2)+0.02*(setpoint::duty-100) - data::v_rms*2; //// ERROR CALCULATION calibrated 10/05/2017
   //PI_V::err = 0.2*(setpoint::voltage/2)+0.008*setpoint::duty - data::v_rms;
   
  //PI_V::err = setpoint::voltage  - (6.33*data::v_rms + 1.57);
  //PI_V::err = 0.35*setpoint::voltage/2 - data::v_rms*2; //// ERROR CALCULATION calibrated 09/26/2017
  actuate_inputs();
  PI_V::I = PI_V::I + PI_V::err*(ts-t_prev)*1e-3;
  
  // build the string from the data
  String mystring = "";
  mystring += ts;
  mystring += ',';
  mystring += setpoint::voltage;
  mystring += ',';
  mystring += setpoint::frequency;
  mystring += ',';
  mystring += setpoint::flowrate;
  mystring += ',';
 // mystring += setpoint::dist;
 // mystring += ',';
  mystring += location::cur_loc/10;
  mystring += ',';
  mystring += setpoint::duty;
  //mystring += data::dist;
  //mystring += ',';
  mystring += data::photo_a;
  mystring += ',';
  mystring += data::photo_b;
  mystring += ',';
  mystring += data::v_rms;
  mystring += ',';
  mystring += data::t_emb;
  mystring += ',';
  mystring += data::i_rms;
  mystring += ',';
  mystring += location::cur_x;
  mystring += ',';
  mystring += location::cur_y;
  // calculate the CRC8 of the string
  // first, convert string to array of chars (signed ints)
  char mychars[mystring.length()+1]; 
  mystring.toCharArray(mychars,sizeof(mychars));
  // then, convert chars into unsigned ints
  uint8_t myuints[sizeof(mychars)];
  for (int i=0; i < sizeof(mychars); i++) {
    myuints[i] = (uint8_t) mychars[i];
  }
  // finally, calculate the crc of the unsigned ints
  int mycrc = crc8(myuints,sizeof(myuints));

  // printing the data string
  Serial.print(mystring);
  Serial.print(',');
  // printing the crc
  Serial.print(mycrc);
  //Serial.print(',');
  //Serial.print(ts-t_prev);
  Serial.println();
  delay(LOOPDELAY);
  t_prev=ts;
  /*
  Serial.print(ts);
  Serial.print(",\t");
  Serial.print(setpoint::voltage);
  Serial.print(",\t");
  Serial.print(setpoint::frequency);
  Serial.print(",\t");
  Serial.print(setpoint::flowrate);
  Serial.print(",\t");
  Serial.print(data::dist); // print distance
  Serial.print(",\t");i
  Serial.print(data::photo_a); // print optical intensity
  Serial.print(",\t");
  Serial.print(data::photo_b); // print optical intensity
  Serial.print(",\t");
  if (proxsensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  */
}

