/*
24-774 Advanced Control System Integration
*/

#include "Aero.h"

#ifndef ACSI_aero_lib_h
#define ACSI_aero_lib_h

extern bool startup = true;  // true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed

extern unsigned long previousMicros;   // used to store the last time the SPI data was written
extern unsigned long currentMicros;    // used to store the current time
extern unsigned long sampleMicros;  // the sample time in microseconds

// global variables for LED intensity (999 is maximum intensity, 0 is off)
extern int LEDRed;
extern int LEDGreen;
extern int LEDBlue;

// Setup global variables for wrap up function
extern float seconds;
extern float pitch;  // pitch angle in radians
extern float yaw;  // yaw angle in radians

extern float motor0Voltage;
extern float motor1Voltage;

extern float currentSense0;
extern float currentSense1;
extern int baseModuleID;
extern int coreModuleID;

// set pin 10 as the slave select for the Quanser Aero
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin;

// initialize the SPI data to be written to the Base Node
extern byte baseMode;                  // normal mode = 1
extern byte baseWriteMask;     // Bxxxxx111 to enable the LEDs, Bxxx11xxx to enable writes to the Base Node encoders
extern byte LEDRedMSB;                 // red LED command MSB
extern byte LEDRedLSB;                 // red LED command LSB
extern byte LEDGreenMSB;               // green LED command MSB
extern byte LEDGreenLSB;               // green LED command LSB
extern byte LEDBlueMSB;                // blue LED command MSB
extern byte LEDBlueLSB;                // blue LED command LSB
extern byte encoder2ByteSet[3];  // encoder2 is set to this value only when writes are enabled with baseWriteMask
extern byte encoder3ByteSet[3];  // encoder3 is set to this value only when writes are enabled with baseWriteMask

// initialize the SPI data to be read from the Base Node
extern byte baseModuleIDMSB;           // module ID MSB (module ID for the Base Node is 772 decimal)
extern byte baseModuleIDLSB;           // module ID LSB
extern byte encoder2Byte[3];     // encoder2 counts (pitch)
extern byte encoder3Byte[3];     // encoder3 counts (yaw)
extern byte tach2Byte[3];        // tachometer2 (pitch)
extern byte tach3Byte[3];        // tachometer3 (yaw)


// initialize the SPI data to be written to the Core Node
extern byte coreMode;                  // normal mode = 1
extern byte coreWriteMask;     // Bxxxx1111 to enable the motors, Bxx11xxxx to enable writes to the Core Node encoders
extern byte motor0MSBx80;              // motor0 command MSB must be B1xxxxxxx to enable amplifier0
extern byte motor0LSB;                 // motor0 command LSB
extern byte motor1MSBx80;              // motor1 command MSB must be B1xxxxxxx to enable amplifier1
extern byte motor1LSB;                 // motor1 command LSB
extern byte encoder0ByteSet[3];  // encoder0 is set to this value only when writes are enabled with coreWriteMask
extern byte encoder1ByteSet[3];  // encoder1 is set to this value only when writes are enabled with coreWriteMask

// initialize the SPI data to be read from the Core Node
extern byte coreModuleIDMSB;           // module ID MSB (module ID for the Core Node is 775 decimal)
extern byte coreModuleIDLSB;           // module ID LSB
extern byte currentSense0MSB;          // motor0 current sense MSB 
extern byte currentSense0LSB;          // motor0 current sense LSB
extern byte currentSense1MSB;          // motor1 current sense MSB 
extern byte currentSense1LSB;          // motor1 current sense LSB
extern byte tach0extern Byte[3];        // tachometer0
extern byte tach1extern Byte[3];        // tachometer1
extern byte moduleStatus;              // module status (the Quanser Aero sends status when there are no errors)
extern byte encoder0extern Byte[3];     // encoder0 counts
extern byte encoder1extern Byte[3];     // encoder1 counts
extern byte xAccelLSB;                 // X-axis accelerometer LSB
extern byte xAccelMSB;                 // X-axis accelerometer MSB
extern byte yAccelLSB;                 // Y-axis accelerometer LSB
extern byte yAccelMSB;                 // Y-axis accelerometer MSB
extern byte zAccelLSB;                 // Z-axis accelerometer LSB
extern byte zAccelMSB;                 // Z-axis accelerometer MSB
extern byte xGyroLSB;                  // X-axis gyroscope LSB
extern byte xGyroMSB;                  // X-axis gyroscope MSB
extern byte yGyroLSB;                  // Y-axis gyroscope LSB
extern byte yGyroMSB;                  // Y-axis gyroscope MSB
extern byte zGyroLSB;                  // Z-axis gyroscope LSB
extern byte zGyroMSB;                  // Z-axis gyroscope MSB
extern byte reserved0;                 // reserved for future use
extern byte reserved1;                 // reserved for future use
extern byte reserved2;                 // reserved for future use
extern byte reserved3;                 // reserved for future use
extern byte reserved4;                 // reserved for future use

//Setup serial builder
extern Display displayData;

void readSensors();
void driveMotor();
void resetQuanserAero();

#endif
