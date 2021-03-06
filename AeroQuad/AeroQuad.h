/*
  AeroQuad v2.1 - October 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

// Flight Software Version
#define VERSION 2.1

#define BAUD 115200
#define LEDPIN 13
#define ON 1
#define OFF 0

#ifdef AeroQuadMega_v2  
  #define LED2PIN 4
  #define LED3PIN 31
#endif
#ifdef AeroQuad_v18
  #define LED2PIN 12
  #define LED3PIN 12
#endif

// Basic axis definitions
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define AUX2 6
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
#define HEADING 5
#define LEVELGYROROLL 6
#define LEVELGYROPITCH 7
#define ALTITUDE 8
#define ZDAMPENING 9
#define SONAR 10

// PID Variables
struct PIDdata {
  float P, I, D;
  float lastPosition;
  float integratedError;
  //float windupGuard; // Thinking about having individual wind up guards for each PID
} PID[11];
// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
// SONAR = 10 (used for sonar hold)
float windupGuard; // Read in from EEPROM

// Smoothing filter parameters
#define GYRO 0
#define ACCEL 1
#define FINDZERO 50
float smoothHeading;

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5
#define AZPIN 12 // Auto zero pin for IDG500 gyros

// Motor control variables
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define MOTORID1 0		
#define MOTORID2 1		
#define MOTORID3 2		
#define MOTORID4 3		
#define MOTORID5 4		
#define MOTORID6 5
#define MINCOMMAND 1000
#define MAXCOMMAND 2000
#if defined(plusConfig) || defined(XConfig)
  #define LASTMOTOR 4
#endif
#if defined(HEXACOAXIAL) || defined(HEXARADIAL)
  #define LASTMOTOR 6
#endif
byte motor;

// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
// If you don't have a DMM use the following:
// AeroQuad Shield v1.7, aref = 3.0
// AeroQuad Shield v1.6 or below, aref = 2.8
float aref; // Read in from EEPROM
int axis;

// Flight Mode
#define ACRO 0
#define STABLE 1
byte flightMode;
int minAcro; // Read in from EEPROM, defines min throttle during flips

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelOff; // Read in from EEPROM
// Scale to convert 1000-2000 PWM to +/- 45 degrees
float mLevelTransmitter = 0.09;
float bLevelTransmitter = -135;

// Heading hold
byte headingHoldConfig;
//float headingScaleFactor;
float heading = 0; // measured heading from yaw gyro (process variable)
float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
float relativeHeading = 0; // current heading the quad is set to (set point)
float absoluteHeading = 0;;
float setHeading = 0;
float commandedYaw = 0;

// Altitude Hold
#define TEMPERATURE 0
#define PRESSURE 1
int throttleAdjust = 0;
int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;
float holdAltitude;
float zDampening;
byte storeAltitude = OFF;
byte altitudeHold = OFF;

// Sonar Hold
int sonarThrottleAdjust = 0;
int sonarMinThrottleAdjust = -150;
int sonarMaxThrottleAdjust = 150;
float sonarHoldAltitude;
byte sonarStoreAltitude = OFF;
byte sonarHold = OFF;

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
#define LASTCHANNEL 7
byte channel;
int delta;

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

// Flight angle variables
float timeConstant;
/*float rateRadPerSec(byte axis);
float rateDegPerSec(byte axis);
float angleDeg(byte axis);
*/

// Camera stabilization variables
// Note: stabilization camera software is still under development
#ifdef Camera
  #define ROLLCAMERAPIN 8
  #define PITCHCAMERAPIN 13
  // map +/-90 degrees to 1000-2000
  float mCamera = 5.556;
  float bCamera = 1500;
  Servo rollCamera;
  Servo pitchCamera;
#endif

// ESC Calibration
byte calibrateESC = 0;
int testCommand = 1000;

// Communication
char queryType = 'X';
byte tlmType = 0;
char string[32];
byte armed = OFF;
byte safetyCheck = OFF;
byte update = 0;

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
#define RECEIVERLOOPTIME 20
#define TELEMETRYLOOPTIME 100
#define FASTTELEMETRYTIME 10
#define CONTROLLOOPTIME 2
#define CAMERALOOPTIME 20
#define AILOOPTIME 2
#define COMPASSLOOPTIME 100
#define ALTITUDELOOPTIME 26
#define SONARLOOPTIME 70

float AIdT = AILOOPTIME / 1000.0;
float controldT = CONTROLLOOPTIME / 1000.0;
float G_Dt = 0.02;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long receiverTime = 0;
unsigned long telemetryTime = 50; // make telemetry output 50ms offset from receiver check
unsigned long sensorTime = 0;
unsigned long controlLoopTime = 1; // offset control loop from analog input loop by 1ms
unsigned long cameraTime = 10;
unsigned long fastTelemetryTime = 0;
unsigned long autoZeroGyroTime = 0;
unsigned long compassTime = 25;
unsigned long altitudeTime = 0;
unsigned long sonarTime=0;

/**************************************************************/
/********************** Debug Parameters **********************/
/**************************************************************/
// Enable/disable control loops for debug
//#define DEBUG
byte receiverLoop = ON;
byte telemetryLoop = ON;
byte sensorLoop = ON;
byte controlLoop = ON;
byte cameraLoop = ON; // Note: stabilization camera software is still under development, moved to Arduino Mega
byte fastTransfer = OFF; // Used for troubleshooting
byte testSignal = LOW;
byte sonarLoop = ON;
// Measured test signal with an oscilloscope:
// All loops on = 2.4 ms
// Analog Input and Control loop on = 2.0 ms
// Analog Input loop on = 1.6 ms
// Analog Input loop on = 1 ms (no comp filter)
// Analog Input loop on = 0.6 ms (comp filter only)
// Control loop on = 0.4 ms
// Receiver loop = 0.4 ms
// Telemetry loop = .04 ms (with no telemetry transmitted)
// Fast Telemetry Transfer (sending 12 bytes = 1.1 ms, sending 14 bytes = 1.3 ms, sending 16 bytes = 1.45 ms, sending 18 bytes = 1.625 ms) 2 bytes = 0.175 ms


// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define LEVEL_PGAIN_ADR 12
#define LEVEL_IGAIN_ADR 16
#define LEVEL_DGAIN_ADR 20
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define LEVELLIMIT_ADR 40
#define LEVELOFF_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
#define MODESMOOTH_ADR 76
#define ROLLSMOOTH_ADR 80
#define PITCHSMOOTH_ADR 84
#define YAWSMOOTH_ADR 88
#define THROTTLESMOOTH_ADR 92
#define GYRO_ROLL_ZERO_ADR 96
#define GYRO_PITCH_ZERO_ADR 100
#define GYRO_YAW_ZERO_ADR 104
#define PITCH_PGAIN_ADR 124
#define PITCH_IGAIN_ADR 128
#define PITCH_DGAIN_ADR 132
#define LEVEL_PITCH_PGAIN_ADR 136
#define LEVEL_PITCH_IGAIN_ADR 140
#define LEVEL_PITCH_DGAIN_ADR 144
#define THROTTLESCALE_ADR 148
#define THROTTLEOFFSET_ADR 152
#define ROLLSCALE_ADR 156
#define ROLLOFFSET_ADR 160
#define PITCHSCALE_ADR 164
#define PITCHOFFSET_ADR 168
#define YAWSCALE_ADR 172
#define YAWOFFSET_ADR 176
#define MODESCALE_ADR 180
#define MODEOFFSET_ADR 184
#define AUXSCALE_ADR 188
#define AUXOFFSET_ADR 192
#define AUXSMOOTH_ADR 196
#define HEADINGSMOOTH_ADR 200
#define HEADING_PGAIN_ADR 204
#define HEADING_IGAIN_ADR 208
#define HEADING_DGAIN_ADR 212
#define AREF_ADR 216
#define FLIGHTMODE_ADR 220
#define LEVEL_GYRO_ROLL_PGAIN_ADR 224
#define LEVEL_GYRO_ROLL_IGAIN_ADR 228
#define LEVEL_GYRO_ROLL_DGAIN_ADR 232
#define LEVEL_GYRO_PITCH_PGAIN_ADR 236
#define LEVEL_GYRO_PITCH_IGAIN_ADR 240
#define LEVEL_GYRO_PITCH_DGAIN_ADR 244
#define HEADINGHOLD_ADR 248
#define MINACRO_ADR 252
#define ACCEL1G_ADR 256
#define ALTITUDE_PGAIN_ADR 260
#define ALTITUDE_IGAIN_ADR 264
#define ALTITUDE_DGAIN_ADR 268
#define ALTITUDE_MAX_THROTTLE_ADR 272
#define ALTITUDE_MIN_THROTTLE_ADR 276
#define ALTITUDE_SMOOTH_ADR  280
#define ZDAMP_PGAIN_ADR 284
#define ZDAMP_IGAIN_ADR 288
#define ZDAMP_DGAIN_ADR 292

int findMode(int *data, int arraySize); // defined in Sensors.pde
float arctan2(float y, float x); // defined in Sensors.pde
float readFloat(int address); // defined in DataStorage.h
void writeFloat(float value, int address); // defined in DataStorage.h
void readEEPROM(void); // defined in DataStorage.h
void readPilotCommands(void); // defined in FlightCommand.pde
void readSensors(void); // defined in Sensors.pde
void flightControl(void); // defined in FlightControl.pde
void readSerialCommand(void);  //defined in SerialCom.pde
void sendSerialTelemetry(void); // defined in SerialCom.pde
void printInt(int data); // defined in SerialCom.pde
float readFloatSerial(void); // defined in SerialCom.pde
void comma(void); // defined in SerialCom.pde
int findMode(int *data, int arraySize); // defined in Sensors.pde

