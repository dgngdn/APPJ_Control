/* http://arduino.cc/en/Hacking/LibraryTutorial */
/* http://ww1.microchip.com/downloads/en/DeviceDoc/22250A.pdf */

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

void setup()
{
  Serial.begin(9600);
  SPI.begin();
}

void loop()
{
  for(int i = 0 ; i < DAC_STEPS / step_interval; i++)
  {
   DAC.Set(DAC_STEPS-1-i*step_interval, i*step_interval);
   delay(10);
   
   data::DAC_A = analogRead(PIN_DAC_A);
   data::DAC_B = analogRead(PIN_DAC_B);
   data::DAC_SUM = data::DAC_A + data::DAC_B;
   
   Serial.print(data::DAC_A);
   Serial.print(",");
   Serial.print(data::DAC_B);
   Serial.print(",");
   Serial.print(data::DAC_SUM);
   Serial.println();
  }
}

