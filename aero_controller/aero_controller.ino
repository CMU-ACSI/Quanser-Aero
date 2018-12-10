/*
Quanser Aero Startup - ACSI

An example using the Arduino UNO board to communicate with the Quanser Aero
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

#include "Aero.h"  // Quanser Aero library
#include "ACSI_aero_lib.h"

/* DECLARE GLOBAL VARIABLES HERE IF NEEDED */
float t = 0;

// Discrete-time controller
void controller_step() {  /* MODIFY THIS FUNCTION */
  // Accessible global variables:
  //    - Input variables
  //        float seconds;  // Time elapsed since start of program in seconds
  //        float pitch;    // Aero pitch in radians
  //        float yaw;      // Aero yaw in radians
  //    - Output variables -
  //        float motor0Voltage;  // Signal sent to motor0 in Volts
  //        float motor1Voltage;  // Signal sent to motor1 in Volts
  //        int LEDRed;           // Red LED intensity on a scale of 0 to 999
  //        int LEDGreen;         // Green LED intensity on a scale of 0 to 999
  //        int LEDBlue;          // Blue LED intensity on a scale of 0 to 999

  // Below demonstrates changing the LED state (you probably don't care) 
  // and changing the motor voltage (you certainly DO care)
  t = int(seconds) % 12;
  
  if (t < 2){
    motor0Voltage = 0.0;
    motor1Voltage = 0.0;

    LEDRed = 0;
    LEDGreen = 999;
    LEDBlue = 0;
    
  } else if (t >= 2 && t < 4){
    motor0Voltage = 10.0;
    motor1Voltage = 0.0;

    LEDRed = 0;
    LEDGreen = 0;
    LEDBlue = 999;

  } else if (t >= 4 && t < 6){
    motor0Voltage = -10.0;
    motor1Voltage = 0.0;

    LEDRed = 0;
    LEDGreen = 500;
    LEDBlue = 999;

  } else if (t >= 6 && t < 8){
    motor0Voltage = 0.0;
    motor0Voltage = 0.0;
    
    LEDRed = 0;
    LEDGreen = 999;
    LEDBlue = 0;

  } else if (t >= 8 && t < 10 ){
    motor0Voltage = 0.0;
    motor1Voltage = 10.0;

    LEDRed = 999;
    LEDGreen = 0;
    LEDBlue = 0;

  } else if (t >= 10 && t < 12){
    motor0Voltage = 0.0;
    motor1Voltage = -10.0;

    LEDRed = 999;
    LEDGreen = 500;
    LEDBlue = 0;
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

  // Initialize global variables 
  /* CHANGE HYPERPARAMETERS HERE */
  sampleMicros = 2000; // set the sample time in microseconds

  /* ADDITIONAL INITIALIZATIONS HERE IF NEEDED (ex: GENERATE REFERENCE OFFLINE) */

}

// This function will be called repeatedly until Arduino is reset
void loop() {
  // Reset after the Arduino power is cycled or the reset pushbutton is pressed
  if (startup) {
    resetQuanserAero();
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
    sampleTimeTickCount++;
    if (sampleTimeTickCount == serialOutputDecimation) {
      displayData.buildString(pitch*(180.0/M_PI), yaw*(180.0/M_PI), currentSense0, currentSense1);
      sampleTimeTickCount = 0;
    }
    
    // Run discrete controller timestep
    controller_step();

    // Write the data to the Qube servo
    driveMotor();
  }

  // Print data to the Arduino Serial Monitor in between SPI transactions
  // (Note that the Serial.print() function is time consuming.  Printing the entire
  // string at once would exceed the sample time required to balance the aero.)
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

      // if there is something to read, get the character
      if (Serial.available() > 0) {
        char currentChar = Serial.read();
        
        if (currentChar == 'r') {
          resetStall = 1;              
        }
      }
    }
  }
}
