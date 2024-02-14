//
//    FILE: HX_calibration.ino
//  AUTHOR: Rob Tillaart (Edited by Niklas M. Melton)
// PURPOSE: HX711 demo
//     URL: https://github.com/RobTillaart/HX711


#include "HX711.h"

HX711 myScale;

uint8_t dataPin = 3;
uint8_t clockPin = 2;

uint32_t start, stop;
volatile float f;


void setup()
{
  Serial.begin(9600);
  Serial.println(__FILE__);
  Serial.print("LIBRARY VERSION: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  myScale.begin(dataPin, clockPin);
}

void loop()
{
  calibrate();
}



void calibrate()
{
  Serial.println("\n\nCALIBRATION\n===========");
  Serial.println("Remove all weight from the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("and press enter\n");
  while (Serial.available() == 0);

  // Serial.println("Determine zero weight offset");
  Serial.println("Scale Tare... Please Wait");
  myScale.tare(20);  // average 20 measurements.
  // uint32_t offset = myScale.get_offset();

  // Serial.print("OFFSET: ");
  // Serial.println(offset);
  // Serial.println();


  Serial.println("Place a weight on the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("Enter the weight in (whole) grams and press enter");
  uint32_t weight = 0;
  while (Serial.peek() != '\n')
  {
    if (Serial.available())
    {
      char ch = Serial.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight = weight + (ch - '0');
      }
    }
  }
  Serial.print("WEIGHT: ");
  Serial.println(weight);
  myScale.calibrate_scale(weight, 20);
  float scale = myScale.get_scale();

  Serial.print("SCALE_CAL_FACTOR:  ");
  Serial.println(scale, 6);

  // Serial.print("\nuse scale.set_offset(");
  // Serial.print(offset);
  // Serial.print("); and scale.set_scale(");
  // Serial.print(scale, 6);
  // Serial.print(");\n");
  // Serial.println("in the setup of your project");

  Serial.println("\n\n");

  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println("Press enter to rerun calibration\n");
  while (Serial.available() == 0);
  
}


//  -- END OF FILE --
