/*
  Project: Automated Arduino Thrust Tester
  File:    [auto_thrust_tester.ino]
  Author:  Niklas M. Melton
  Date:    2024-02-05
  License: Creative Commons Attribution-NonCommercial 4.0 International License
           (CC BY-NC 4.0)

  Description:
  A simple and automated thrust testing solution for EDFs and brushed or brushless motors.

  This code is licensed under the Creative Commons Attribution-NonCommercial 4.0 International License.
  You are free to:
  - Share — copy and redistribute the material in any medium or format
  - Adapt — remix, transform, and build upon the material

  Under the following terms:
  - Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  - NonCommercial — You may not use the material for commercial purposes.

  For the full license text, visit: https://creativecommons.org/licenses/by-nc/4.0/

  Disclaimer:
  This code is provided as-is without any warranty. The author is not responsible
  for any damages or liabilities that may arise from its use.
*/

// ===========================================================================================================================
//                                        Define behavior in this section
// ===========================================================================================================================

// Scale Calibration Factor
// ************************ RUN THE CALIBRATION SCRIPT AND SET THIS VALUE AS INSTRUCTED ************************
#define SCALE_CAL_FACTOR 204.2

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur. 2MHz is slow but reliable
#define SPI_SPEED SD_SCK_MHZ(2)
// sample rate in milliseconds
#define SAMPLE_MS 250 //4Hz

// pin settings
#define SD_CS_PIN 4
#define BUTTON_PIN 7
#define SCALE_DATA_PIN 3
#define SCALE_CLOCK_PIN 2
#define SERVO_PIN 8

/* 
  Enable automated testing with servo control
  If this is defined, we will first test the motor to see if we have control.
  if the motor is present, the test will proceed automatically.
  if no control is perceived, the test will proceed in manual mode.
  Keep this defined if you want the option for either. 
*/
#define AUTO_TEST // comment out if you only want manual testing
#define MIN_TEST_FORCE 50 // minimum force required for perceiving ESC control
#define TIME_AT_MAX_THROTTLE 3000 //time in milliseconds spent at maximum throttle. Should be a multiple of SAMPLE_MS.

// #define CONT_SYNC // enable continuous syncing of the written data to the sd. 
//    Constant syncing seems to cause errors for some cards.

// #define DEBUG_MODE // enable verbose serial output

// ===========================================================================================================================
// ===========================================================================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <LiquidCrystal_PCF8574.h>
#include "HX711.h"
#include <Servo.h>

// Function Declarations
void setupLcd();
void setupCard();
void setupScale();
void setupButton();
void setupServo();
void rampServo(int setPos, int dt);
bool checkServo();
void writeHeader();
void writeData(unsigned long time, int pos, float force);
void getFileName(const char* dirName, char* newFileName);
bool isMatchingFormat(const char* fileName);
int getFileNum(const char* fileName);
void delayCheckButton(const unsigned long dt, bool full = true);
void printLcd(const char* line1 = "", const char* line2 = "");
void printLcdScroll(const char* inputText);
void startTest();
void endTest();
void stepTest();


// Global Variables
LiquidCrystal_PCF8574 lcd(0x27);
SdFat SD; // requires the card to be formatted with FAT16 and be <=2gb
File file;
HX711 scale;
volatile float unitForce = 0.0;
float maxForce = 0.0;
int buttonState = HIGH;
volatile bool continueFlag = true;
Servo myServo;
int servoPos = 0;
short servoStep = 0;
bool scaleActive = false;
#ifdef AUTO_TEST
  bool servoChecked = false;
  bool servoActive = true;
#else
  bool servoChecked = true;
  bool servoActive = false;
#endif
const int maxServoPos = 180 + (TIME_AT_MAX_THROTTLE / SAMPLE_MS) ;
bool cardExists = false;
unsigned long scaleStartTime = 0;
unsigned long readTime = 0;
char activeFileName[10]; // Fixed-size buffer for activeFileName
int maxFileNum = 0;


// Function Definitions

void setupLcd() {
  /**
 * Instantiate the LCD screen and print "initializing..." to the screen
 */
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  printLcd("Initializing...");
  delay(500);
}


void printLcd(const char* line1, const char* line2) {
  /**
 * print two lines of text to the LCD screen. Both lines are optional.
 * printing anything to the first line will clear the screen but printing
 * only to the second line will not.
 * If DEBUG_MODE is defined, these lines will also be printed to the Serial console
 *
 * @param line1 some text to print on the first line
 * @param line2 some text to print to the second line
 */

  if (line1 != NULL && line1[0] != '\0') {
    lcd.clear();
    #ifdef DEBUG_MODE
      Serial.println(line1);
    #endif
    lcd.setCursor(0, 0);
    lcd.print(line1);
  }

  if (line2 != NULL && line2[0] != '\0') {
    #ifdef DEBUG_MODE
      Serial.println(line2);
    #endif
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
}


void printLcdScroll(const char* inputText) {
  /**
 * print n-many lines to the LCD screen. 
 * Line will be split on the newline "\n" character
 * Each pair of two lines will be displayed for 2 seconds unless the button is pressed
 * If DEBUG_MODE is defined, these lines will also be printed to the Serial console
 *
 * @param inputText some text to scroll where lines are separated with "\n" 
 */

  char* textCopy = strdup(inputText); // Make a copy of the input text

  // Split the text on newline characters and display two segments at a time
  char* token = strtok(textCopy, "\n");
  while (token != NULL) {
    printLcd(token); // Display the current segment
    token = strtok(NULL, "\n"); // Get the next segment
    
    if (token != NULL) {
      printLcd("", token); // Display the next segment
      token = strtok(NULL, "\n"); // Get the next segment
    }

    delayCheckButton(2000, false); // Wait for 2 seconds or until button pressed
  }

  free(textCopy); // Free the memory allocated for the copy of inputText
}


void delayCheckButton(const unsigned long dt, bool full) {
  /**
  * Delay for up to dt milliseconds while checking the button state
  * This function will always exit after a maximum of dt ms
  * if full == true, wait full time regardless of button state
  * if full == false, exit as soon as button has been pressed AND released
  * @param dt maximum time to delay in milliseconds 
  * @param full always wait for dt milliseconds no matter what
  */

  unsigned long total = 0;
  buttonState = HIGH;
  // wait for dt to expire or until button is pressed
  while (total < dt && continueFlag) {
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == LOW) {
      continueFlag = false;
      break;
    } else {
      delay(min(50, dt-total));
      total += 50;
    }
  }
  // continue waiting until button is depressed or until full time has expired if full == true
  while (total < dt && (full || buttonState == LOW)) {
    buttonState = digitalRead(BUTTON_PIN);
    if (buttonState == HIGH) {
      return;
    } else {
      delay(min(50, dt-total));
      total += 50;
    }
  }
}


void setupCard() {
  /**
  * Instantiate the SD card and handle failures
  */
  #ifdef DEBUG_MODE
    printLcd("Init SD Card...");
    delay(500);
  #endif
  if (SD.begin(SD_CS_PIN, SPI_SPEED)) {
    continueFlag = false;
    cardExists = true;
  }
  while (continueFlag) {
    printLcdScroll("Card Failed or\nNot Present\nPress Button\nto Continue");
  }
  continueFlag = true;
}


void setupScale() {
  /**
  * Instantiate the Load cell (scale) 
  */
  #ifdef DEBUG_MODE
    printLcd("Init Scale...");
    delay(500);
  #endif
  scale.begin(SCALE_DATA_PIN, SCALE_CLOCK_PIN);
  scale.set_scale(SCALE_CAL_FACTOR);
  scale.tare();
}


void setupButton() {
  /**
  * Instantiate the push button
  */
  #ifdef DEBUG_MODE
    printLcd("Init Button...");
    delay(500);
  #endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}


void setupServo() {
  /**
  * Instantiate the servo (ESC control)
  */
  #ifdef DEBUG_MODE
    printLcd("Init Servo...");
    delay(500);
  #endif
  myServo.attach(SERVO_PIN);
}


void rampServo(int setPos, int dt) {
  /**
  * Ramp the servo to the setPos with a dt time-step between steps
  * Step size is 1
  * @param setPos final position of servo in degrees 
  * @param dt time delay in milliseconds between steps 
  */
  int deltaPos = setPos - servoPos;
  int step = (deltaPos < 0) ? -1 : 1;
  while (servoPos != setPos && continueFlag) {
    servoPos += step;
    myServo.write(servoPos);
    delayCheckButton(dt, false);
  }
}


bool checkServo() {
  /**
  * Check if the ESC is connected by ramping and measuring force changes
  */
  continueFlag = true;
  printLcd("Checking Servo","Please Wait");
  delay(2000);
  printLcd("Press Button","to Skip");
  delayCheckButton(2000, false);
  float startForce = scale.get_units(10);
  // begin increasing motor speed
  rampServo(90, 60);
  // hold for 1 second
  delayCheckButton(1000, false);
  // measure force
  unitForce = scale.get_units(10);
  // begin decreasing motor speed
  rampServo(0, 60);
  // ensure we end at 0
  servoPos = 0;
  myServo.write(servoPos);
  // check result
  if (!continueFlag) {
    printLcd("Skipped!");
    delay(1000);
    return false;
  }
  if (abs(unitForce - startForce) > MIN_TEST_FORCE) {
    printLcd("Servo Found!");
    delay(1000);
    return true;
  } else {
    printLcd("Servo Not Found!");
    delay(1000);
    return false;
  }
  continueFlag = true;
}


void writeHeader() {
  /**
  * Write a header to the data file in csv format
  */
  file.print(F("millis,servo_pos,force"));
  file.println();
}


void writeData(unsigned long time, int pos, float force) {
  /**
  * Write data to the data file in csv format
  * @param time measurement time in milliseconds from start
  * @param pos servo position at time of measurement
  * @param force force measured at time
  */
  file.print(String(time));
  file.print(F(","));
  file.print(String(pos));
  file.print(F(","));
  file.print(String(force));
  file.println();
}


void getFileName(const char* dirName, char* newFileName) {
  /**
  * Get a file name for the data file
  * Crawl the directory and look for files with the format t_X.csv where X is an integer and 0 <= X <= 999
  * Find the highest value of X and return a new file name in the format of t_Y.csv where Y = X+1
  * The crawl only happens once and the results are cached
  * If X = 999, Ask user if they want to restart at 1, potentially overwriting old data
  * The first file name returned by this function will be t_1.csv if the directory is empty
  * @param dirName directory where data is stored
  * @param newFileName file name buffer
  */
  
  if (maxFileNum == 0) { // we only need to crawl the directory once if we cache the result
    File file;
    if (file.open(dirName, O_READ)) {
      #ifdef DEBUG_MODE
        Serial.println("Files in directory: " + String(dirName));
      #endif

      while (true) {
        File entry;
        if (!entry.openNext(&file, O_READ)) {
          // No more files in the directory
          break;
        }

        if (!entry.isDir()) {
          char fileName[10]; // Maximum file name length in 7.3 format
          entry.getName(fileName, sizeof(fileName));
          if (isMatchingFormat(fileName)) {
            #ifdef DEBUG_MODE
              Serial.println("\t" + String(fileName));
            #endif
            int num = getFileNum(fileName);
            maxFileNum = max(num, maxFileNum);
          }
        }

        entry.close();
      }

      file.close();
    } else {
      file.close();
      #ifdef DEBUG_MODE
        Serial.println("Error opening directory: " + String(dirName));
      #endif
    }
  }
  int nextFileNum = maxFileNum + 1;
  maxFileNum = int(nextFileNum);
  if (nextFileNum > 999) {
    delay(500);
    continueFlag = true;
    while (continueFlag) {
      printLcdScroll("File number\ntoo high!\nClear SD Card\nor\nPress button\nto reset at t_1");
    }
    continueFlag = true;
    nextFileNum = 1;
    maxFileNum = 1;
  }
  snprintf(newFileName, 10, "t_%d.csv", nextFileNum);
}


bool isMatchingFormat(const char* fileName) {
  /**
  * Check if filename has the format t_X.csv where X is an integer and 0 <= X <= 999
  * @param fileName file name buffer
  * @return boolean that is true if the fileName matches the correct format
  */
  // Check if the file name matches the format 't_X.csv'
  int nameLength = strlen(fileName);
  if (nameLength < 7 || nameLength > 10) {
    return false; // File name length should be between 7 and 10 characters
  }

  if (strncmp(fileName, "t_", 2) != 0 || strncmp(fileName + nameLength - 4, ".csv", 4) != 0) {
    return false; // Check for 't_X.csv' format
  }

  // Check if the characters between 't_' and '.csv' are all digits
  for (int i = 2; i < nameLength - 4; i++) {
    if (!isdigit(fileName[i])) {
      return false; // Check if X is a number
    }
  }

  return true;
}


int getFileNum(const char* fileName) {
  /**
  * Get the number from a filename in the format t_X.csv where X is an integer and 0 <= X <= 999
  * @param fileName file name buffer
  * @return int the value of X in the file name
  */

  int fileNum = 0;
  const char* numStart = fileName + 2;
  const char* numEnd = strstr(fileName, ".csv");

  if (numStart < numEnd) {
    char numString[10];
    int j = 0;

    for (const char* i = numStart; i < numEnd; i++) {
      numString[j++] = *i;
    }
    numString[j] = '\0';

    fileNum = atoi(numString);
  }
  return fileNum;
}


void startTest() {
  /**
  * Initialize the testing parameters, check the ESC for control, and print updates to the LCD
  */
  servoStep = 2;
  if (cardExists) {
    getFileName("/", activeFileName);
    if (!file.open(activeFileName, FILE_WRITE)) {
      printLcd("Failed to Open ", activeFileName);
    } else {
      writeHeader();
    }
  }
  printLcd("Test Starting...");
  delay(3000);
  if (!servoChecked && servoActive) {
    servoChecked = checkServo();
    if (servoChecked) {
      printLcd("Sweep Starting...");
      delay(2000);
    } else {
      printLcd("Ready for Input!");
      delay(2000);
    }
  }
  scaleStartTime = millis();
  readTime = 0;
}


void endTest() {
  /**
  * Reset the testing parameters, close the data file, and print updates to the LCD
  */
  if (cardExists) {
    file.close();
  }
  printLcd("Test Complete.");
  delay(2000);
  if (cardExists) {
    printLcd("Data Saved to:", activeFileName);
  }
  maxForce = 0.0; // Reset maxForce to zero
}


void stepTest() {
  /**
  * Step the test by taking measurements at constant intervals and incrementing servo (ESC) position
  * If CONT_SYNC is defined, this function will ensure data is continuously synced to the SD card (risky on slow cards)
  */
  char forceString[10];
  char maxString[10];
  char forceStringTrimmed[17]; // Fixed-size buffer for forceStringTrimmed
  char maxStringTrimmed[17]; // Fixed-size buffer for maxStringTrimmed

  unsigned long waitRemaining = SAMPLE_MS - min(200, (millis() - scaleStartTime) - readTime);
  if (waitRemaining > 0) {
    delayCheckButton(waitRemaining);
  }

  readTime = millis() - scaleStartTime;
  unitForce = scale.get_units(1);
  maxForce = max(unitForce, maxForce);

  // only display decimal if force magnitude is less than 100g
  if (abs(unitForce) < 100) {
    dtostrf(unitForce, 4, 1, forceString);
  } else {
    dtostrf(unitForce, 4, 0, forceString);
  }
  if (abs(maxForce) < 100) {
    dtostrf(maxForce, 4, 1, maxString);
  } else {
    dtostrf(maxForce, 4, 0, maxString);
  }
  

  snprintf(forceStringTrimmed, sizeof(forceStringTrimmed), "Force: %s g", forceString);
  snprintf(maxStringTrimmed, sizeof(maxStringTrimmed), "Max: %s g", maxString);

  printLcd(forceStringTrimmed, maxStringTrimmed);

  int actualServoPos = min(180, servoPos);

  if (cardExists) {
    writeData(readTime, actualServoPos, unitForce);
    
    #ifdef CONT_SYNC
      // Force data to SD and update the directory entry to avoid data loss.
      if (!file.sync() || file.getWriteError()) {
        printLcd("Write Error", "Aborting Test");
        delay(5000);
        continueFlag = false;
        if (servoChecked && servoActive) {
          servoStep = 0;
          servoPos = 0;
          myServo.write(servoPos);
        }
        return;
      }
    #endif
  }

  if (servoChecked && servoActive) {
    servoPos += servoStep;
    myServo.write(actualServoPos);

    if (servoPos >= maxServoPos) {
      servoStep = -2;
    } else if (servoPos <= 0) {
      servoStep = 0;
      continueFlag = false;
    }
  }
}


void setup() {
  // setup everything
  #ifdef DEBUG_MODE
    Serial.begin(9600);
    delay(2000);
    Serial.println();
  #endif
  
  setupLcd();
  setupButton();
  setupCard();
  setupScale();
  #ifdef AUTO_TEST
    setupServo();
  #endif
}

void loop() {
  // main loop
  while (continueFlag && !scaleActive) {
    printLcdScroll("Ready to Test!\n \nPress Button\nto Begin");
  }

  if (!continueFlag) {
    scaleActive = !scaleActive;
    continueFlag = true;
    if (scaleActive) {
      startTest();
    } else {
      endTest();
      delay(5000);
    }
    continueFlag = true;
  }

  if (scaleActive && continueFlag) {
    stepTest();
  } else{
    delayCheckButton(250, false);
  }
}
