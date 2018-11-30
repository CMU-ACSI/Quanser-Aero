/*
Quanser Aero

An example using the Arduino UNO board to communicate with the Quanser Aero through
the microcontroller interface panel.  This code is meant to serve as a template you
can use as a starting point for your own Aero code.

Select 115200 baud on the Arduino Serial Monitor.
Tested with Arduino IDE 1.6.8 (arduino.cc) and Arduino IDE 1.7.10 (arduino.org).

Created 2016 by Quanser Inc.
www.quanser.com
*/

// include the Quanser Aero library
#include "Aero.h"

// include the SPI library and the math library
#include <SPI.h>
#include <math.h>

bool startup = true;  // true the first time the sketch is run after the Arduino power is cycled or the reset pushbutton is pressed

unsigned long previousMicros = 0;  // used to store the last time the SPI data was written
const long sampleTime = 2000;  // set the sample time (the interval between SPI transactions) to 2000us = 2ms

unsigned long sampleTimeTickCount = 0;
const long serialOutputDecimation = 600;  // number of samples between serial outputs

int resetStall = 0;
float motor0Voltage = 0;
float motor1Voltage = 0;

// set pin 10 as the slave select for the Quanser Aero
// (Note that if a different pin is used for the slave select, pin 10 should be set as
// an output to prevent accidentally putting the Arduino UNO into slave mode.)
const int slaveSelectPin = 10;

// initialize the SPI data to be written to the Base Node
byte baseMode = 1;                  // normal mode = 1
byte baseWriteMask = B00000111;     // Bxxxxx111 to enable the LEDs, Bxxx11xxx to enable writes to the Base Node encoders
byte LEDRedMSB = 0;                 // red LED command MSB
byte LEDRedLSB = 0;                 // red LED command LSB
byte LEDGreenMSB = 0;               // green LED command MSB
byte LEDGreenLSB = 0;               // green LED command LSB
byte LEDBlueMSB = 0;                // blue LED command MSB
byte LEDBlueLSB = 0;                // blue LED command LSB
byte encoder2ByteSet[3] = {0,0,0};  // encoder2 is set to this value only when writes are enabled with baseWriteMask
byte encoder3ByteSet[3] = {0,0,0};  // encoder3 is set to this value only when writes are enabled with baseWriteMask

// initialize the SPI data to be read from the Base Node
byte baseModuleIDMSB = 0;           // module ID MSB (module ID for the Base Node is 772 decimal)
byte baseModuleIDLSB = 0;           // module ID LSB
byte encoder2Byte[3] = {0,0,0};     // encoder2 counts (pitch)
byte encoder3Byte[3] = {0,0,0};     // encoder3 counts (yaw)
byte tach2Byte[3] = {0,0,0};        // tachometer2 (pitch)
byte tach3Byte[3] = {0,0,0};        // tachometer3 (yaw)


// initialize the SPI data to be written to the Core Node
byte coreMode = 1;                  // normal mode = 1
byte coreWriteMask = B00001111;     // Bxxxx1111 to enable the motors, Bxx11xxxx to enable writes to the Core Node encoders
byte motor0MSB = 0x80;              // motor0 command MSB must be B1xxxxxxx to enable amplifier0
byte motor0LSB = 0;                 // motor0 command LSB
byte motor1MSB = 0x80;              // motor1 command MSB must be B1xxxxxxx to enable amplifier1
byte motor1LSB = 0;                 // motor1 command LSB
byte encoder0ByteSet[3] = {0,0,0};  // encoder0 is set to this value only when writes are enabled with coreWriteMask
byte encoder1ByteSet[3] = {0,0,0};  // encoder1 is set to this value only when writes are enabled with coreWriteMask

// initialize the SPI data to be read from the Core Node
byte coreModuleIDMSB = 0;           // module ID MSB (module ID for the Core Node is 775 decimal)
byte coreModuleIDLSB = 0;           // module ID LSB
byte currentSense0MSB = 0;          // motor0 current sense MSB 
byte currentSense0LSB = 0;          // motor0 current sense LSB
byte currentSense1MSB = 0;          // motor1 current sense MSB 
byte currentSense1LSB = 0;          // motor1 current sense LSB
byte tach0Byte[3] = {0,0,0};        // tachometer0
byte tach1Byte[3] = {0,0,0};        // tachometer1
byte moduleStatus = 0;              // module status (the Quanser Aero sends status = 0 when there are no errors)
byte encoder0Byte[3] = {0,0,0};     // encoder0 counts
byte encoder1Byte[3] = {0,0,0};     // encoder1 counts
byte xAccelLSB = 0;                 // X-axis accelerometer LSB
byte xAccelMSB = 0;                 // X-axis accelerometer MSB
byte yAccelLSB = 0;                 // Y-axis accelerometer LSB
byte yAccelMSB = 0;                 // Y-axis accelerometer MSB
byte zAccelLSB = 0;                 // Z-axis accelerometer LSB
byte zAccelMSB = 0;                 // Z-axis accelerometer MSB
byte xGyroLSB = 0;                  // X-axis gyroscope LSB
byte xGyroMSB = 0;                  // X-axis gyroscope MSB
byte yGyroLSB = 0;                  // Y-axis gyroscope LSB
byte yGyroMSB = 0;                  // Y-axis gyroscope MSB
byte zGyroLSB = 0;                  // Z-axis gyroscope LSB
byte zGyroMSB = 0;                  // Z-axis gyroscope MSB
byte reserved0 = 0;                 // reserved for future use
byte reserved1 = 0;                 // reserved for future use
byte reserved2 = 0;                 // reserved for future use
byte reserved3 = 0;                 // reserved for future use
byte reserved4 = 0;                 // reserved for future use

// global variables for LED intensity (999 is maximum intensity, 0 is off)
int LEDRed = 0;
int LEDGreen = 0;
int LEDBlue = 0;
// ============= Global variables LQR controller ==============
// Control gain K used here was taken from simulink demo model and tested in a rev C Aero on Oct. 28. 2016
//
float desired[4]={0,0,0,0};  /// 0 => Pitch, 1 ==> yaw
float Error[4]={0,0,0,0};
float StateX[4]={0,0,0,0};
float gainP[4]={98.2088,-103.0645,32.2643,-29.075};
float gainY[4]={156.3469,66.1643,45.5122,17.1068};
float Vmotors[2]={0,0};
int milisecs=0;
float seconds=0;
float Pitch_n_k1=0;
float Pitch_dot_k1=0;
float Yaw_n_k1=0;
float Yaw_dot_k1=0;
float Pitch_deg=0;
float Yaw_deg=0;
//============================================================
//Setup serial builder
Display displayData;

void setup() {
  // set the slaveSelectPin as an output
  pinMode (slaveSelectPin, OUTPUT);
  
  // initialize SPI
  SPI.begin();
  
  // initialize serial communication at 2500000 baud
  // (Note that 250000 baud must be selected from the drop-down list on the Arduino
  // Serial Monitor for the data to be displayed properly.)
  Serial.begin(250000);
}

void loop() {
  
  // after the Arduino power is cycled or the reset pushbutton is pressed, call the resetQuanserAero function
  if (startup) {

  //  resetQuanserAero();
    startup = false;
    LEDBlue = 999;

  }


  // if the difference between the current time and the last time an SPI transaction
  // occurred is greater than the sample time, start a new SPI transaction
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= sampleTime) {
    previousMicros = previousMicros + sampleTime;  

    //==============Setpoint Clock ===============
        milisecs++; 
        SetpointGen();  // Creates different setpoits for Pitch and Yaw angles
    //============================================
    
    // initialize the SPI bus using the defined speed, data order and data mode
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
    // take the slave select pin low to select the device
    digitalWrite(slaveSelectPin, LOW);
    
    // send and receive the Base Node data via SPI
    baseModuleIDMSB = SPI.transfer(baseMode);            // read the module ID MSB, send the mode
    baseModuleIDLSB = SPI.transfer(0);                   // read the module ID LSB
    encoder2Byte[2] = SPI.transfer(baseWriteMask);       // read encoder2 byte 2, send the write mask
    encoder2Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder2 byte 1, send the red LED MSB
    encoder2Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder2 byte 0, send the red LED LSB
    encoder3Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder3 byte 2, send the green LED MSB
    encoder3Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder3 byte 1, send the green LED LSB
    encoder3Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder3 byte 0, send the blue LED MSB
    tach2Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer2 byte 2, send the blue LED LSB
    tach2Byte[1] = SPI.transfer(encoder2ByteSet[2]);     // read tachometer2 byte 1, send encoder2 byte 2
    tach2Byte[0] = SPI.transfer(encoder2ByteSet[1]);     // read tachometer2 byte 0, send encoder2 byte 1
    tach3Byte[2] = SPI.transfer(encoder2ByteSet[0]);     // read tachometer3 byte 2, send encoder2 byte 0
    tach3Byte[1] = SPI.transfer(encoder3ByteSet[2]);     // read tachometer3 byte 1, send encoder3 byte 2
    tach3Byte[0] = SPI.transfer(encoder3ByteSet[1]);     // read tachometer3 byte 0, send encoder3 byte 1
    SPI.transfer(encoder3ByteSet[0]);                    // send encoder3 byte 0
    
    // send and receive the Core Node data via SPI
    coreModuleIDMSB = SPI.transfer(coreMode);            // read the module ID MSB, send the mode
    coreModuleIDLSB = SPI.transfer(0);                   // read the module ID LSB
    currentSense0MSB = SPI.transfer(coreWriteMask);      // read motor0 current sense MSB, send the write mask
    currentSense0LSB = SPI.transfer(motor0MSB);          // read motor0 current sense LSB, send motor0 command MSB
    currentSense1MSB = SPI.transfer(motor0LSB);          // read motor1 current sense MSB, send motor0 command LSB
    currentSense1LSB = SPI.transfer(motor1MSB);          // read motor1 current sense LSB, send motor1 command MSB
    tach0Byte[2] = SPI.transfer(motor1LSB);              // read tachometer0 byte 2, send motor1 command LSB
//    tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2       ==> commented out for code reduction
//    tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
//    tach1Byte[2] = SPI.transfer(encoder0ByteSet[0]);     // read tachometer1 byte 2, send encoder0 byte 0
//    tach1Byte[1] = SPI.transfer(encoder1ByteSet[2]);     // read tachometer1 byte 1, send encoder1 byte 2
//    tach1Byte[0] = SPI.transfer(encoder1ByteSet[1]);     // read tachometer1 byte 0, send encoder1 byte 1
//    moduleStatus = SPI.transfer(encoder1ByteSet[0]);     // read the status, send encoder1 byte 0
//    encoder0Byte[2] = SPI.transfer(0);                   // read encoder0 byte 2
//    encoder0Byte[1] = SPI.transfer(0);                   // read encoder0 byte 1
//    encoder0Byte[0] = SPI.transfer(0);                   // read encoder0 byte 0
//    encoder1Byte[2] = SPI.transfer(0);                   // read encoder1 byte 2
//    encoder1Byte[1] = SPI.transfer(0);                   // read encoder1 byte 1
//    encoder1Byte[0] = SPI.transfer(0);                   // read encoder1 byte 0
//    xAccelLSB = SPI.transfer(0);                         // read X-axis accelerometer LSB
//    xAccelMSB = SPI.transfer(0);                         // read X-axis accelerometer MSB
//    yAccelLSB = SPI.transfer(0);                         // read Y-axis accelerometer LSB
//    yAccelMSB = SPI.transfer(0);                         // read Y-axis accelerometer MSB
//    zAccelLSB = SPI.transfer(0);                         // read Z-axis accelerometer LSB
//    zAccelMSB = SPI.transfer(0);                         // read Z-axis accelerometer MSB
//    xGyroLSB = SPI.transfer(0);                          // read X-axis gyroscope LSB
//    xGyroMSB = SPI.transfer(0);                          // read X-axis gyroscope MSB
//    yGyroLSB = SPI.transfer(0);                          // read Y-axis gyroscope LSB
//    yGyroMSB = SPI.transfer(0);                          // read Y-axis gyroscope MSB
//    zGyroLSB = SPI.transfer(0);                          // read Z-axis gyroscope LSB
//    zGyroMSB = SPI.transfer(0);                          // read Z-axis gyroscope MSB
//    reserved0 = SPI.transfer(0);                         // reserved for future use
//    reserved1 = SPI.transfer(0);                         // reserved for future use
//    reserved2 = SPI.transfer(0);                         // reserved for future use
//    reserved3 = SPI.transfer(0);                         // reserved for future use
//    reserved4 = SPI.transfer(0);                         // reserved for future use
    
    // take the slave select pin high to de-select the device
    digitalWrite(slaveSelectPin, HIGH);
    SPI.endTransaction();
    
    // combine the received bytes to assemble the sensor values
    
    // Module IDs
    int baseModuleID = (baseModuleIDMSB << 8) | baseModuleIDLSB;
    int coreModuleID = (coreModuleIDMSB << 8) | coreModuleIDLSB;
    
    //Encoder Counts
//    long encoder0 = ((long)encoder0Byte[2] << 16) | ((long)encoder0Byte[1] << 8) | encoder0Byte[0];  // motor 0 angle
//    if (encoder0 & 0x00800000) {
//      encoder0 = encoder0 | 0xFF000000;
//    }
//    long encoder1 = ((long)encoder1Byte[2] << 16) | ((long)encoder1Byte[1] << 8) | encoder1Byte[0];  // motor 1 angle
//    if (encoder1 & 0x00800000) {
//      encoder1 = encoder1 | 0xFF000000;
//    }
    long encoder2 = ((long)encoder2Byte[2] << 16) | ((long)encoder2Byte[1] << 8) | encoder2Byte[0];  // pitch
    if (encoder2 & 0x00800000) {
      encoder2 = encoder2 | 0xFF000000;
    }
    long encoder3 = ((long)encoder3Byte[2] << 16) | ((long)encoder3Byte[1] << 8) | encoder3Byte[0];  // yaw
    if (encoder3 & 0x00800000) {
      encoder3 = encoder3 | 0xFF000000;
    }
    
    // convert encoder counts to degrees
  //  float encoder0Deg = (float)encoder0 * (360.0 / 2048.0);  // motor 0 angle
  //  float encoder1Deg = (float)encoder1 * (360.0 / 2048.0);  // motor 1 angle
    float encoder2Deg = (float)encoder2 * (360.0 / 2048.0);  // pitch
    float encoder3Deg = (float)encoder3 * (360.0 / 4096.0);  // yaw (encoder3 is higher resolution than the other encoders)
    Pitch_deg=encoder2Deg;
    Yaw_deg=encoder3Deg;
    //Current Sense Values
    float currentSense0 = (currentSense0MSB << 8) | currentSense0LSB;
    float currentSense1 = (currentSense1MSB << 8) | currentSense1LSB;
    
    
    //***********************************Start of Custom Code  for LQR 2DOF controller **********************************************************
    //
    //
    float PitchRad= 0.01743*encoder2Deg;
    float YawRad=0.01743*encoder3Deg;
    StateX[0]=PitchRad;
    StateX[1]=YawRad;
      //**** Z transfor  1st order derrivative filter   start *******************
    float Pitch_n=PitchRad;
   // float Pitch_dot=(47.62*Pitch_n)-(47.62*Pitch_n_k1)+(0.9048*Pitch_dot_k1);
   float Pitch_dot=(46*Pitch_n)-(46*Pitch_n_k1)+(0.839*Pitch_dot_k1);
    StateX[2]=Pitch_dot;
    Pitch_n_k1=Pitch_n;
    Pitch_dot_k1=Pitch_dot;
    float Yaw_n=YawRad;
    float Yaw_dot=(46*Yaw_n)-(46*Yaw_n_k1)+(0.839*Yaw_dot_k1);
    StateX[3]=Yaw_dot;
    Yaw_n_k1=Yaw_n;
    Yaw_dot_k1=Yaw_dot; 
    Error[0]=desired[0]-StateX[0];
    Error[1]=desired[1]-StateX[1];
    AeroGains();                                // Calculates Voltage for both motors
    motor0Voltage = Vmotors[0];
    motor1Voltage = Vmotors[1]; 
 //*****************************************************end*************************************************************************************
 //
 // set the saturation limit to +/- 20V for Motor0  as it is in Simulink model. 
 //
      if (motor0Voltage > 24.0) {
        motor0Voltage = 24.0;
      }
      else if (motor0Voltage < -24.0) {
        motor0Voltage = -24.0;
      }
// set the saturation limit to +/- 15V for Motor1
      if (motor1Voltage > 24.0) {
        motor1Voltage = 24.0;
      }
      else if (motor1Voltage < -24.0) {
        motor1Voltage = -24.0;
      }

    
    //**************************************End of Custom Code ****************************************************
    
    // convert the LED intensities to MSB and LSB
    LEDRedMSB = (byte)(LEDRed >> 8);
    LEDRedLSB = (byte)(LEDRed & 0x00FF);
    LEDGreenMSB = (byte)(LEDGreen >> 8);
    LEDGreenLSB = (byte)(LEDGreen & 0x00FF);
    LEDBlueMSB = (byte)(LEDBlue >> 8);
    LEDBlueLSB = (byte)(LEDBlue & 0x00FF);
    
    // convert the analog value to the PWM duty cycle that will produce the same average voltage
    float motor0PWM = motor0Voltage * (625.0 / 15.0);
    float motor1PWM = motor1Voltage * (625.0 / 15.0);

    if (resetStall) {
      motor0LSB = 0;
      motor0MSB = 0;

      motor1LSB = 0;
      motor1MSB = 0;

      LEDRed = 999;
      LEDBlue = 0;                
      
      resetStall--; // reset clears when the value == 0.
      
    } else {
      int motor0 = (int)motor0PWM;  // convert float to int (2 bytes)
      motor0 = motor0 | 0x8000;  // motor0 command MSB must be B1xxxxxxx to enable amplifier0
      motor0MSB = (byte)(motor0 >> 8);
      motor0LSB = (byte)(motor0 & 0x00FF);
      
      int motor1 = (int)motor1PWM;  // convert float to int (2 bytes)
      motor1 = motor1 | 0x8000;  // motor1 command MSB must be B1xxxxxxx to enable amplifier1
      motor1MSB = (byte)(motor1 >> 8);     
      motor1LSB = (byte)(motor1 & 0x00FF);
     
    }

    
    sampleTimeTickCount++;

    if (sampleTimeTickCount == serialOutputDecimation) {

  displayData.buildString(Pitch_deg, Yaw_deg,currentSense0, currentSense1);//, encoder2, encoder3);//Pitch_deg, Yaw_deg);
      sampleTimeTickCount = 0;
      
    }
      

  }
  // print data to the Arduino Serial Monitor in between SPI transactions
  // (Note that the Serial.print() function is time consuming.  Printing the entire
  // string at once would exceed the sample time required to balance the pendulum.)
  else {
    // only print if there's a string ready to be printed, and there's enough time before the next SPI transaction
    if ( (displayData.dDataReady) && (currentMicros - previousMicros <= (sampleTime - 100)) ) {
      // if there is room available in the serial buffer, print one character
      //
      // **************************** Serial comm disabled ====   start 
      if(Serial.availableForWrite() > 0) {
        Serial.print(displayData.dData[displayData.dDataIndex]);
        displayData.dDataIndex = displayData.dDataIndex + 1;
        // if the entire string has been printed, clear the flag so a new string can be obtained
        if(displayData.dDataIndex == displayData.dData.length()) {
          displayData.dDataReady = false;
        }
      }
      ///     
      // if there is something to read, get the character
      if (Serial.available() > 0) {
        char currentChar = Serial.read();
        
        if (currentChar == 'r') {
          resetStall = 1;              
        } 
      }
//////  ********************     ==========  serial comm disables ====
      
    }
  }
}

// This function is used to clear the stall errors and reset the encoder values to 0.
// The motors and LEDs are turned off when this function is called.
void resetQuanserAero() {
  
  // enable the motors and LEDs, and enable writes to the encoders
  baseWriteMask = B00011111;
  coreWriteMask = B00111111;
  
  // turn off the LEDs
  LEDRedMSB = 0;
  LEDRedLSB = 0;
  LEDGreenMSB = 0;
  LEDGreenLSB = 0;
  LEDBlueMSB = 0;
  LEDBlueLSB = 0;
  
  // reset the encoder values to 0
//  encoder0ByteSet[2] = 0;   ==> code reduction
//  encoder0ByteSet[1] = 0;
//  encoder0ByteSet[0] = 0;
//  encoder1ByteSet[2] = 0;
//  encoder1ByteSet[1] = 0;
//  encoder1ByteSet[0] = 0;
  encoder2ByteSet[2] = 0;
  encoder2ByteSet[1] = 0;
  encoder2ByteSet[0] = 0;
  encoder3ByteSet[2] = 0;
  encoder3ByteSet[1] = 0;
  encoder3ByteSet[0] = 0;
  
  // turn off the motors, and clear the stall errors by disabling the amplifiers
  motor0MSB = 0;  // motor0 command MSB is B0xxxxxxx to disable amplifier0
  motor0LSB = 0;
  motor1MSB = 0;  // motor1 command MSB is B0xxxxxxx to disable amplifier1
  motor1LSB = 0;
  
  // initialize the SPI bus using the defined speed, data order and data mode
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  digitalWrite(slaveSelectPin, LOW);   // take the slave select pin low to select the device
  
  // send and receive the Base Node data via SPI
  baseModuleIDMSB = SPI.transfer(baseMode);            // read the module ID MSB, send the mode
  baseModuleIDLSB = SPI.transfer(0);                   // read the module ID LSB
  encoder2Byte[2] = SPI.transfer(baseWriteMask);       // read encoder2 byte 2, send the write mask
  encoder2Byte[1] = SPI.transfer(LEDRedMSB);           // read encoder2 byte 1, send the red LED MSB
  encoder2Byte[0] = SPI.transfer(LEDRedLSB);           // read encoder2 byte 0, send the red LED LSB
  encoder3Byte[2] = SPI.transfer(LEDGreenMSB);         // read encoder3 byte 2, send the green LED MSB
  encoder3Byte[1] = SPI.transfer(LEDGreenLSB);         // read encoder3 byte 1, send the green LED LSB
  encoder3Byte[0] = SPI.transfer(LEDBlueMSB);          // read encoder3 byte 0, send the blue LED MSB
  tach2Byte[2] = SPI.transfer(LEDBlueLSB);             // read tachometer2 byte 2, send the blue LED LSB
  tach2Byte[1] = SPI.transfer(encoder2ByteSet[2]);     // read tachometer2 byte 1, send encoder2 byte 2
  tach2Byte[0] = SPI.transfer(encoder2ByteSet[1]);     // read tachometer2 byte 0, send encoder2 byte 1
  tach3Byte[2] = SPI.transfer(encoder2ByteSet[0]);     // read tachometer3 byte 2, send encoder2 byte 0
  tach3Byte[1] = SPI.transfer(encoder3ByteSet[2]);     // read tachometer3 byte 1, send encoder3 byte 2
  tach3Byte[0] = SPI.transfer(encoder3ByteSet[1]);     // read tachometer3 byte 0, send encoder3 byte 1
  SPI.transfer(encoder3ByteSet[0]);                    // send encoder3 byte 0
  
  // send and receive the Core Node data via SPI
  coreModuleIDMSB = SPI.transfer(coreMode);            // read the module ID MSB, send the mode
  coreModuleIDLSB = SPI.transfer(0);                   // read the module ID LSB
  currentSense0MSB = SPI.transfer(coreWriteMask);      // read motor0 current sense MSB, send the write mask
  currentSense0LSB = SPI.transfer(motor0MSB);          // read motor0 current sense LSB, send motor0 command MSB
  currentSense1MSB = SPI.transfer(motor0LSB);          // read motor1 current sense MSB, send motor0 command LSB
  currentSense1LSB = SPI.transfer(motor1MSB);          // read motor1 current sense LSB, send motor1 command MSB
  tach0Byte[2] = SPI.transfer(motor1LSB);              // read tachometer0 byte 2, send motor1 command LSB
//  tach0Byte[1] = SPI.transfer(encoder0ByteSet[2]);     // read tachometer0 byte 1, send encoder0 byte 2  ==>code reduction
//  tach0Byte[0] = SPI.transfer(encoder0ByteSet[1]);     // read tachometer0 byte 0, send encoder0 byte 1
//  tach1Byte[2] = SPI.transfer(encoder0ByteSet[0]);     // read tachometer1 byte 2, send encoder0 byte 0
//  tach1Byte[1] = SPI.transfer(encoder1ByteSet[2]);     // read tachometer1 byte 1, send encoder1 byte 2
//  tach1Byte[0] = SPI.transfer(encoder1ByteSet[1]);     // read tachometer1 byte 0, send encoder1 byte 1
//  moduleStatus = SPI.transfer(encoder1ByteSet[0]);     // read the status, send encoder1 byte 0
//  encoder0Byte[2] = SPI.transfer(0);                   // read encoder0 byte 2
//  encoder0Byte[1] = SPI.transfer(0);                   // read encoder0 byte 1
//  encoder0Byte[0] = SPI.transfer(0);                   // read encoder0 byte 0
//  encoder1Byte[2] = SPI.transfer(0);                   // read encoder1 byte 2
//  encoder1Byte[1] = SPI.transfer(0);                   // read encoder1 byte 1
//  encoder1Byte[0] = SPI.transfer(0);                   // read encoder1 byte 0
//  xAccelLSB = SPI.transfer(0);                         // read X-axis accelerometer LSB
//  xAccelMSB = SPI.transfer(0);                         // read X-axis accelerometer MSB
//  yAccelLSB = SPI.transfer(0);                         // read Y-axis accelerometer LSB
//  yAccelMSB = SPI.transfer(0);                         // read Y-axis accelerometer MSB
//  zAccelLSB = SPI.transfer(0);                         // read Z-axis accelerometer LSB
//  zAccelMSB = SPI.transfer(0);                         // read Z-axis accelerometer MSB
//  xGyroLSB = SPI.transfer(0);                          // read X-axis gyroscope LSB
//  xGyroMSB = SPI.transfer(0);                          // read X-axis gyroscope MSB
//  yGyroLSB = SPI.transfer(0);                          // read Y-axis gyroscope LSB
//  yGyroMSB = SPI.transfer(0);                          // read Y-axis gyroscope MSB
//  zGyroLSB = SPI.transfer(0);                          // read Z-axis gyroscope LSB
//  zGyroMSB = SPI.transfer(0);                          // read Z-axis gyroscope MSB
//  reserved0 = SPI.transfer(0);                         // reserved for future use
//  reserved1 = SPI.transfer(0);                         // reserved for future use
//  reserved2 = SPI.transfer(0);                         // reserved for future use
//  reserved3 = SPI.transfer(0);                         // reserved for future use
//  reserved4 = SPI.transfer(0);                         // checksum
  
  digitalWrite(slaveSelectPin, HIGH);  // take the slave select pin high to de-select the device
  SPI.endTransaction();
  
  baseWriteMask = B00000111;  // enable the LEDs, disable writes to the Base Node encoders
  coreWriteMask = B00001111;  // enable the motors, disable writes to the Core Node encoders
  
  motor0MSB = 0x80;  // enable amplifier0
  motor1MSB = 0x80;  // enable amplifier1
}


