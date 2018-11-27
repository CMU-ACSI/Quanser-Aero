/*
Quanser Aero Header

Created 2016 by Quanser Inc.
www.quanser.com
*/

#ifndef Aero_h
#define Aero_h

#include <Arduino.h>

class Display
{
public:
	Display(); //Build the display class
	//void buildString(float encoder0Deg, float encoder1Deg, float encoder2Deg, float encoder3Deg, float currentSense0, float currentSense1, int baseModuleID, int coreModuleID, int moduleStatus); //Create the data string
	void buildString(float encoder2Deg, float encoder3Deg, float currentSense0, float currentSense1); //Create the data string
 //	void buildString( float currentSense0, float currentSense1); //Create the data string
   
	String dData;  // string that will be printed to the Arduino Serial Monitor
	boolean dDataReady;  // true when there is a string ready to be printed
	int dDataIndex;  // used to print the string one character at a time
private:
	void reset(int baseModuleID, int coreModuleID, int moduleStatus); //reset the data string
};
#endif
