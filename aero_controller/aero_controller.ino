/*
Quanser QUBE Servo Startup - ACSI

An example using the Arduino UNO board to communicate with the Quanser QUBE Servo
through the microcontroller interface panel.  This implements a pseudo-controller
to demonstrate sensor read and motor write functionality.

Select 250000 baud on the Arduino Serial Tested with Arduino Software (IDE) 1.6.7.

Original file created 2016 by Quanser Inc.
www.quanser.com

Modified for ACSI 2017 by Bin Xu
www.cmu.edu

Modified for ACSI 2018 by Austin Wang
www.cmu.edu
*/

#include <SPI.h>
#include <math.h>

#include "QUBEServo.h"  // QUBE Servo library
#include "ACSI_lib.h"

// DECLARE GLOBAL VARIABLES HERE IF NEEDED

// Discrete-time controller (MODIFY THIS FUNCTION)
void controller_step() {
  // Accessible global variables:
  //    Input variables:
  //        seconds - (float) Time elapsed since start of program in seconds
  //        theta - (float) Pendulum angle radians
  //        alpha - (float) Arm angle in radians
  //    Output variables:
  //        motorVoltage - (float) Signal sent to the motor in Volts
  //        LEDRed - (float) Red LED intensity on a scale of 0 to 999
  //        LEDGreen - (float) Green LED intensity on a scale of 0 to 999
  //        LEDBlue - (float) Blue LED intensity on a scale of 0 to 999

  // Below demonstrates changing the LED state (you probably don't care) 
  // and changing the motor voltage (you certainly DO care)
  if (theta <= -(20*M_PI/180.0)) {
    LEDRed = 0;
    LEDGreen = 999;
    LEDBlue = 0;
    motorVoltage = -1.0;
  } else if (theta >= (20*M_PI/180.0)) {
    LEDRed = 0;
    LEDGreen = 0;
    LEDBlue = 999;
    motorVoltage = 1.0;
  }
}

// This function will be called once during initialization
void setup() {
  // Set the slaveSelectPin as an output
  pinMode(slaveSelectPin, OUTPUT);
  
  // Initialize SPI
  SPI.begin();
  
  // Initialize serial communication at 250000 baud
  // (Note that 250000 baud must be selected from the drop-down list on the Arduino
  // Serial Monitor for the data to be displayed properly.)
  Serial.begin(250000);

  // Initialize global variables (CHANGE HYPERPARAMETERS HERE)
  sampleMicros = 2000; // set the sample time in microseconds

  // ADDITIONAL INITIALIZATIONS HERE IF NEEDED (ex: GENERATE REFERENCE OFFLINE)

}

// This function will be called repeatedly until Arduino is reset
void loop() {
  // Reset after the Arduino power is cycled or the reset pushbutton is pressed
  if (startup) {
    resetQUBEServo();
    startup = false;
  }
  
  // If the difference between the current time and the last time an SPI transaction
  // occurred is greater than the sample time, start a new SPI transaction
  // (alternatively, use a timer interrupt)
  currentMicros = micros();
  if (currentMicros - previousMicros >= sampleMicros) {
    previousMicros = previousMicros + sampleMicros;

    // Read data into global variables. For variable definitions, see above comments.
    readSensors();

    // This will define the data that will be displayed at the serial terminal.
    displayData.buildString(theta, alpha, currentSense, moduleID, moduleStatus);

    // Run discrete controller timestep
    controller_step();

    // Write the data to the Qube servo
    driveMotor();
  }

  // Print data to the Arduino Serial Monitor in between SPI transactions
  // (Note that the Serial.print() function is time consuming.  Printing the entire
  // string at once would exceed the sample time required to balance the pendulum.)
  else {  //We're in between samples
    // Only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
    if ( (displayData.dDataReady) && (currentMicros - previousMicros <= (sampleMicros - 100)) ) {
      // If there is room available in the serial buffer, print one character
      if(Serial.availableForWrite() > 0) {
        Serial.print(displayData.dData[displayData.dDataIndex]);
        displayData.dDataIndex = displayData.dDataIndex + 1;
        // If the entire string has been printed, clear the flag so a new string can be obtained
        if(displayData.dDataIndex == displayData.dData.length()) {
          displayData.dDataReady = false;
        }
      }
    }
  }
}
