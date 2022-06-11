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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
void readSerialCommand() {
  // Check for serial message
  if (DebugPort.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = DebugPort.read();
    switch (queryType) {
     case 'z' :
      DebugPort.print(receiver.getRaw(ROLL));
      DebugPort.print("\t");
      DebugPort.print(receiver.getRaw(AUX));
      DebugPort.print("\t");
      DebugPort.println(receiver.getRaw(AUX2));
      break;
      
     case 'y': // Receive roll and pitch gyro PID
      PID[SONAR].P = readFloatSerial();
      DebugPort.println(PID[SONAR].P);
      break;
      
    case 'u': // Receive roll and pitch gyro PID
      PID[SONAR].D = readFloatSerial();
      DebugPort.println(PID[SONAR].D);
      break;
      
    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = readFloatSerial();
      PID[ROLL].I = readFloatSerial();
      PID[ROLL].D = readFloatSerial();
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = readFloatSerial();
      PID[PITCH].I = readFloatSerial();
      PID[PITCH].D = readFloatSerial();
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      minAcro = readFloatSerial();
      break;
    case 'C': // Receive yaw PID
      PID[YAW].P = readFloatSerial();
      PID[YAW].I = readFloatSerial();
      PID[YAW].D = readFloatSerial();
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      PID[HEADING].P = readFloatSerial();
      PID[HEADING].I = readFloatSerial();
      PID[HEADING].D = readFloatSerial();
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
      headingHoldConfig = readFloatSerial();
      heading = 0;
      relativeHeading = 0;
      headingHold = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = readFloatSerial();
      PID[LEVELROLL].I = readFloatSerial();
      PID[LEVELROLL].D = readFloatSerial();
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = readFloatSerial();
      PID[LEVELPITCH].I = readFloatSerial();
      PID[LEVELPITCH].D = readFloatSerial();
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      PID[LEVELGYROROLL].P = readFloatSerial();
      PID[LEVELGYROROLL].I = readFloatSerial();
      PID[LEVELGYROROLL].D = readFloatSerial();
      PID[LEVELGYROROLL].lastPosition = 0;
      PID[LEVELGYROROLL].integratedError = 0;
      PID[LEVELGYROPITCH].P = readFloatSerial();
      PID[LEVELGYROPITCH].I = readFloatSerial();
      PID[LEVELGYROPITCH].D = readFloatSerial();
      PID[LEVELGYROPITCH].lastPosition = 0;
      PID[LEVELGYROPITCH].integratedError = 0;
      windupGuard = readFloatSerial();
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receiver altitude hold PID
      #ifdef AltitudeHold
        PID[ALTITUDE].P = readFloatSerial();
        PID[ALTITUDE].I = readFloatSerial();
        PID[ALTITUDE].D = readFloatSerial();
        PID[ALTITUDE].lastPosition = 0;
        PID[ALTITUDE].integratedError = 0;
        minThrottleAdjust = readFloatSerial();
        maxThrottleAdjust = readFloatSerial();
        altitude.setSmoothFactor(readFloatSerial());
        PID[ZDAMPENING].P = readFloatSerial();
        PID[ZDAMPENING].I = readFloatSerial();
        PID[ZDAMPENING].D = readFloatSerial();
      #endif
      break;
    case 'K': // Receive data filtering values
      gyro.setSmoothFactor(readFloatSerial());
      accel.setSmoothFactor(readFloatSerial());
      timeConstant = readFloatSerial();
      //flightMode = readFloatSerial();
      break;
    case 'M': // Receive transmitter smoothing values
      receiver.setXmitFactor(readFloatSerial());
      receiver.setSmoothFactor(ROLL, readFloatSerial());
      receiver.setSmoothFactor(PITCH, readFloatSerial());
      receiver.setSmoothFactor(YAW, readFloatSerial());
      receiver.setSmoothFactor(THROTTLE, readFloatSerial());
      receiver.setSmoothFactor(MODE, readFloatSerial());
      receiver.setSmoothFactor(AUX, readFloatSerial());
      break;
    case 'O': // Receive transmitter calibration values
      receiver.setTransmitterSlope(ROLL, readFloatSerial());
      receiver.setTransmitterOffset(ROLL, readFloatSerial());
      receiver.setTransmitterSlope(PITCH, readFloatSerial());
      receiver.setTransmitterOffset(PITCH, readFloatSerial());
      receiver.setTransmitterSlope(YAW, readFloatSerial());
      receiver.setTransmitterOffset(YAW, readFloatSerial());
      receiver.setTransmitterSlope(THROTTLE, readFloatSerial());
      receiver.setTransmitterOffset(THROTTLE, readFloatSerial());
      receiver.setTransmitterSlope(MODE, readFloatSerial());
      receiver.setTransmitterOffset(MODE, readFloatSerial());
      receiver.setTransmitterSlope(AUX, readFloatSerial());
      receiver.setTransmitterOffset(AUX, readFloatSerial());
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case 'Y': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      gyro.calibrate();
      accel.calibrate();
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      armed = 0;
      calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      armed = 0;
      calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      armed = 0;
      testCommand = readFloatSerial();
      calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      armed = 0;
      calibrateESC = 0;
      testCommand = 1000;
      break;
    case '5': // Send individual motor commands (motor, command)
      armed = 0;
      calibrateESC = 5;
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setRemoteCommand(motor, readFloatSerial());
      break;
    case 'a':
      // spare
      break;
    case 'b': // calibrate gyros
      gyro.calibrate();
      break;
    case 'c': // calibrate accels
      accel.calibrate();
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    }
  digitalWrite(LEDPIN, HIGH);
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************
void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    DebugPort.print(accel.getZaxis());
    comma();
    DebugPort.print(accel.getOneG());
    comma();
    DebugPort.print(zDampening);
    comma();
    DebugPort.print(throttleAdjust);
    DebugPort.println();
    //queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    DebugPort.print(PID[ROLL].P);
    comma();
    DebugPort.print(PID[ROLL].I);
    comma();
    DebugPort.print(PID[ROLL].D);
    comma();
    DebugPort.print(PID[PITCH].P);
    comma();
    DebugPort.print(PID[PITCH].I);
    comma();
    DebugPort.print(PID[PITCH].D);
    comma();
    DebugPort.println(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    DebugPort.print(PID[YAW].P);
    comma();
    DebugPort.print(PID[YAW].I);
    comma();
    DebugPort.print(PID[YAW].D);
    comma();
    DebugPort.print(PID[HEADING].P);
    comma();
    DebugPort.print(PID[HEADING].I);
    comma();
    DebugPort.print(PID[HEADING].D);
    comma();
    DebugPort.println(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    DebugPort.print(PID[LEVELROLL].P);
    comma();
    DebugPort.print(PID[LEVELROLL].I);
    comma();
    DebugPort.print(PID[LEVELROLL].D);
    comma();
    DebugPort.print(PID[LEVELPITCH].P);
    comma();
    DebugPort.print(PID[LEVELPITCH].I);
    comma();
    DebugPort.print(PID[LEVELPITCH].D);
    comma();
    DebugPort.print(PID[LEVELGYROROLL].P);
    comma();
    DebugPort.print(PID[LEVELGYROROLL].I);
    comma();
    DebugPort.print(PID[LEVELGYROROLL].D);
    comma();
    DebugPort.print(PID[LEVELGYROPITCH].P);
    comma();
    DebugPort.print(PID[LEVELGYROPITCH].I);
    comma();
    DebugPort.print(PID[LEVELGYROPITCH].D);
    comma();
    DebugPort.println(windupGuard);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    DebugPort.print(levelLimit);
    comma();
    DebugPort.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Altitude Hold
    #ifdef AltitudeHold
      DebugPort.print(PID[ALTITUDE].P);
      comma();
      DebugPort.print(PID[ALTITUDE].I);
      comma();
      DebugPort.print(PID[ALTITUDE].D);
      comma();
      DebugPort.print(minThrottleAdjust);
      comma();
      DebugPort.print(maxThrottleAdjust);
      comma();
      DebugPort.print(altitude.getSmoothFactor());
      comma();
      DebugPort.print(PID[ZDAMPENING].P);
      comma();
      DebugPort.print(PID[ZDAMPENING].I);
      comma();
      DebugPort.println(PID[ZDAMPENING].D);
    #else
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.println('0');
    #endif
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    DebugPort.print(gyro.getSmoothFactor());
    comma();
    DebugPort.print(accel.getSmoothFactor());
    comma();
    DebugPort.println(timeConstant);
    // comma();
    // Serial.println(flightMode, DEC);
   queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    DebugPort.print(receiver.getXmitFactor());
    comma();
    for (axis = ROLL; axis < AUX; axis++) {
      DebugPort.print(receiver.getSmoothFactor(axis));
      comma();
    }
    DebugPort.println(receiver.getSmoothFactor(AUX));
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (axis = ROLL; axis < AUX; axis++) {
      DebugPort.print(receiver.getTransmitterSlope(axis));
      comma();
      DebugPort.print(receiver.getTransmitterOffset(axis));
      comma();
    }
    DebugPort.print(receiver.getTransmitterSlope(AUX));
    comma();
    DebugPort.println(receiver.getTransmitterOffset(AUX));
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(gyro.getData(axis));
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(accel.getData(axis));
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      DebugPort.print(levelAdjust[axis]);
      comma();
    }
    DebugPort.print(flightAngle.getData(ROLL));
    comma();
    DebugPort.print(flightAngle.getData(PITCH));
    DebugPort.println();
    break;
  case 'R': // Send raw sensor data
    /*Serial.print(analogRead(ROLLRATEPIN));
    comma();
    Serial.print(analogRead(PITCHRATEPIN));
    comma();
    Serial.print(analogRead(YAWRATEPIN));
    comma();
    Serial.print(analogRead(ROLLACCELPIN));
    comma();
    Serial.print(analogRead(PITCHACCELPIN));
    comma();
    Serial.println(analogRead(ZACCELPIN));*/
    //queryType = 'X';
    break;
  case 'S': // Send all flight data
    DebugPort.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(gyro.getFlightData(axis));
      comma();
    }
    DebugPort.print(receiver.getData(THROTTLE));
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(motors.getMotorAxisCommand(axis));
      comma();
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      DebugPort.print(motors.getMotorCommand(motor));
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(accel.getFlightData(axis));
      comma();
    }
     DebugPort.print(armed, BIN);
    comma();
    if (flightMode == STABLE)
      DebugPort.print(2000);
    if (flightMode == ACRO)
      DebugPort.print(1000);
    #ifdef HeadingMagHold
      comma();
      DebugPort.print(compass.getAbsoluteHeading());
    #else
      comma();
      DebugPort.print('0');
    #endif
    #ifdef AltitudeHold
      comma();
      DebugPort.print(altitude.getData());
      comma();
      DebugPort.print(altitudeHold, DEC);
    #else
      comma();
      DebugPort.print('0');
      comma();
      DebugPort.print('0');
    #endif
    DebugPort.println();
    break;
   case 'T': // Send processed transmitter values
    DebugPort.print(receiver.getXmitFactor());
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      DebugPort.print(receiver.getData(axis));
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      DebugPort.print(levelAdjust[axis]);
      comma();
    }
    DebugPort.print(motors.getMotorAxisCommand(ROLL));
    comma();
    DebugPort.print(motors.getMotorAxisCommand(PITCH));
    comma();
    DebugPort.println(motors.getMotorAxisCommand(YAW));
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (channel = ROLL; channel < AUX; channel++) {
      DebugPort.print(receiver.getData(channel));
      comma();
    }
    DebugPort.println(receiver.getData(AUX));
    break;
  case 'V': // Send receiver status
    for (channel = ROLL; channel < AUX; channel++) {
      DebugPort.print(receiver.getRaw(channel));
      comma();
    }
    DebugPort.println(receiver.getRaw(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    DebugPort.print(receiver.getData(YAW));
    comma();
    DebugPort.print(headingHold);
    comma();
    DebugPort.print(setHeading);
    comma();
    DebugPort.println(relativeHeading);
    break;
  case '6': // Report remote commands
    for (motor = FRONT; motor < LEFT; motor++) {
      DebugPort.print(motors.getRemoteCommand(motor));
      comma();
    }
    DebugPort.println(motors.getRemoteCommand(LEFT));
    break;
  case '!': // Send flight software version
    DebugPort.println(VERSION, 1);
    queryType = 'X';
    break;
  case '#': // Send software configuration
    // Determine which hardware is used to define max/min sensor values for Configurator plots
    #if defined(AeroQuad_v1)
      DebugPort.print('0');
    #elif defined(AeroQuadMega_v1)
      DebugPort.print('1');
    #elif defined(AeroQuad_v18)
      DebugPort.print('2');
    #elif defined(AeroQuadMega_v2)
      DebugPort.print('3');
    #elif defined(AeroQuad_Wii)
      DebugPort.print('4');
    #elif defined(AeroQuadMega_Wii)
      DebugPort.print('5');
    #elif defined(ArduCopter)
      DebugPort.print('6');
    #elif defined(Multipilot)
      DebugPort.print('7');
    #elif defined(MultipilotI2C)
      DebugPort.print('8');
    #endif
    comma();
    // Determine which motor flight configuration for Configurator GUI
    #if defined(plusConfig)
      DebugPort.print('0');
    #elif defined(XConfig)
      DebugPort.print('1');
    #elif defined(HEXACOAXIAL)
      DebugPort.print('2');
    #elif defined(HEXARADIAL)
      DebugPort.print('3');
    #endif
    DebugPort.println();
    queryType = 'X';
    break;  
  case 'e': // Send AREF value
    DebugPort.println(aref);
    queryType = 'X';
    break;
    
  case 'p': // send altimeter
      #ifdef AltitudeHold
      comma();
      DebugPort.print(altitude.getData());
      comma();
      DebugPort.print(altitudeHold, DEC);
      DebugPort.print('0');
      #endif
      break;
      
 case 'q': // send sonar
      #ifdef SonarHold
      DebugPort.println(sonar.range);
      #endif
      break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (DebugPort.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = DebugPort.read();
      timeout = 0;
      index++;
    }
  }  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}


void comma() {
  DebugPort.print(',');
}

void printInt(int data) {
  byte msb, lsb;
  
  msb = data >> 8;
  lsb = data & 0xff;
  
  DebugPort.print(msb, BYTE);
  DebugPort.print(lsb, BYTE);
}
