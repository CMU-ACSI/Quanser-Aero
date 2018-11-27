/*
Quanser Aero Core Library

Created 2016 by Quanser Inc.
www.quanser.com
*/

#include <Arduino.h>
#include "Aero.h"

//Status Bit Masks
const byte motor1StallErrorMask = B00100000;
const byte motor1StallDetectedMask = B00010000;
const byte motor0StallErrorMask = B00001000;
const byte motor0StallDetectedMask = B00000100;
const byte amplifier1FaultMask = B00000010;
const byte amplifier0FaultMask = B00000001;

//Serial Variables
static String dData;  // string that will be printed to the Arduino Serial Monitor
static boolean dDataReady;  // true when there is a string ready to be printed
static int dDataIndex;  // used to print the string one character at a time

//Create the display class
Display::Display()
{
	this->dData = "";
	this->dDataReady = false;
	this->dDataIndex = 0;

	// reserve 256 bytes for the string that will be printed to the Arduino Serial Monitor
	dData.reserve(256);
}


//void Display::buildString(float encoder0Deg, float encoder1Deg, float encoder2Deg, float encoder3Deg, float currentSense0, float currentSense1, int baseModuleID, int coreModuleID, int moduleStatus)
 void Display::buildString(float encoder2Deg, float encoder3Deg, float currentSense0, float currentSense1)
//void Display::buildString(float currentSense0, float currentSense1)
{

	if (this->dDataReady == false) {
		// assemble the string to be printed to the Arduino Serial Monitor
		// (Note that the String() conversion function is time consuming.  If more data
		// needs to displayed than in this example, it may be necessary to assemble the
		// string over multiple sample periods.)

	//	reset(baseModuleID, coreModuleID, moduleStatus);

//    this->dData += "\r\nM0 (deg): ";
//    this->dData += String(round(encoder0Deg));
//
//    this->dData += "\r\nM1 (deg): ";
//   this->dData += String(round(encoder1Deg));

		this->dData += "\r\nPitch(deg): ";
		this->dData += String(round(encoder2Deg));

		this->dData += "\r\nYaw(deg): ";
		this->dData += String(round(encoder3Deg));

    this->dData += "\r\nM0 I(mA): ";
    float currentSense0Amps = (3.3 / (4.0 * 4095.0)) * ((currentSense0 / 2.0) - 4095.0);
    this->dData += String(currentSense0Amps * 1000);

		this->dData += "\r\nM1 I(mA): ";
		float currentSense1Amps = (3.3 / (4.0 * 4095.0)) * ((currentSense1 / 2.0) - 4095.0);
		this->dData += String(currentSense1Amps * 1000);
 //  this->dData += String(seconds);      // ===> just for testing seconds 
		this->dData += "\r\n\n";

		this->dDataReady = true;  // the string is ready to be printed
		this->dDataIndex = 0;
	}
}

void Display::reset(int baseModuleID, int coreModuleID, int moduleStatus)
{
	this->dData = "";  // clear the string

	if (baseModuleID == -1) {
    this->dData += "Base Module ID: No module detected";
  }
  else {
    this->dData += "Base Module ID: ";
    this->dData += String(baseModuleID);
  }

  if (coreModuleID == -1) {
		this->dData += "\r\nCore Module ID: No module detected";
	}
	else {
		this->dData += "\r\nCore Module ID: ";
		this->dData += String(coreModuleID);
	}

  // moduleStatus is 0 when there are no errors
  if (moduleStatus == 0) {
    this->dData += "\r\nStatus: Good";  // '\r' is carriage return and '\n' is new line
  }
  // check for stalls
  else if (moduleStatus & (motor0StallErrorMask | motor0StallDetectedMask | motor1StallErrorMask | motor1StallDetectedMask)) {
    if (moduleStatus & motor0StallErrorMask) {
      this->dData += "\r\nStatus: Motor 0 Stall Error";
    }
    else if (moduleStatus & motor0StallDetectedMask) {
      this->dData += "\r\nStatus: Motor 0 Stall Detected";
    }
    if (moduleStatus & motor1StallErrorMask) {
      this->dData += "\r\nStatus: Motor 1 Stall Error";
    }
    else if (moduleStatus & motor1StallDetectedMask) {
      this->dData += "\r\nStatus: Motor 1 Stall Detected";
    }
  }
  
  // amplifier faults 
  else if (moduleStatus & (amplifier0FaultMask | amplifier1FaultMask)) {
    if (moduleStatus & amplifier0FaultMask) {
      this->dData += "\r\nStatus: Amplifier 0 Fault";
    }
    if (moduleStatus & amplifier1FaultMask) {
      this->dData += "\r\nStatus: Amplifier 1 Fault";
    }
  }
  // if moduleStatus is an unexpected value, print the value
	else {
		this->dData += "\r\nStatus: ";
		this->dData += String(moduleStatus);
	}  
}
