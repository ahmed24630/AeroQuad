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



  #define AUX2SCALE_ADR AUXSCALE_ADR
  #define AUX2OFFSET_ADR AUXOFFSET_ADR
  #define AUX2SMOOTH_ADR AUXSMOOTH_ADR

class  Receiver {
public:
  int receiverData[7];
  int transmitterCommand[7];
  int transmitterCommandSmooth[7];
  int transmitterZero[3];
  int transmitterTrim[3];
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
  float xmitFactor; // Read in from EEPROM
  float transmitterSmooth[7];
  float mTransmitter[7];
  float bTransmitter[7];

  Receiver(void) { 
    transmitterCommand[ROLL] = 1500;
    transmitterCommand[PITCH] = 1500;
    transmitterCommand[YAW] = 1500;
    transmitterCommand[THROTTLE] = 1000;
    transmitterCommand[MODE] = 1000;
    transmitterCommand[AUX] = 1000;
    transmitterCommand[AUX2] = 1000;
    
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = 0;
    for (channel = ROLL; channel < THROTTLE; channel++)
      transmitterZero[channel] = 1500;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void);
  virtual void read(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  
  void _initialize(void) {
    xmitFactor = readFloat(XMITFACTOR_ADR);
    mTransmitter[ROLL] = readFloat(ROLLSCALE_ADR);
    bTransmitter[ROLL] = readFloat(ROLLOFFSET_ADR);
    mTransmitter[PITCH] = readFloat(PITCHSCALE_ADR);
    bTransmitter[PITCH] = readFloat(PITCHOFFSET_ADR);
    mTransmitter[YAW] = readFloat(YAWSCALE_ADR);
    bTransmitter[YAW] = readFloat(YAWOFFSET_ADR);
    mTransmitter[THROTTLE] = readFloat(THROTTLESCALE_ADR);
    bTransmitter[THROTTLE] = readFloat(THROTTLEOFFSET_ADR);
    mTransmitter[MODE] = readFloat(MODESCALE_ADR);
    bTransmitter[MODE] = readFloat(MODEOFFSET_ADR);
    mTransmitter[AUX] = readFloat(AUXSCALE_ADR);
    bTransmitter[AUX] = readFloat(AUXOFFSET_ADR);
    mTransmitter[AUX2] = readFloat(AUX2SCALE_ADR);
    bTransmitter[AUX2] = readFloat(AUX2OFFSET_ADR);
    
    transmitterSmooth[THROTTLE] = readFloat(THROTTLESMOOTH_ADR);
    transmitterSmooth[ROLL] = readFloat(ROLLSMOOTH_ADR);
    transmitterSmooth[PITCH] = readFloat(PITCHSMOOTH_ADR);
    transmitterSmooth[YAW] = readFloat(YAWSMOOTH_ADR);
    transmitterSmooth[MODE] = readFloat(MODESMOOTH_ADR);
    transmitterSmooth[AUX] = readFloat(AUXSMOOTH_ADR);
    transmitterSmooth[AUX2] = readFloat(AUX2SMOOTH_ADR);
  }
  
  const int getRaw(byte channel) {
    return receiverData[channel];
  }
  
  const int getData(byte channel) {
    // reduce sensitivity of transmitter input by xmitFactor
    return transmitterCommand[channel];
  }
  
  const int getTrimData(byte channel) {
    return receiverData[channel] - transmitterTrim[channel];
  }
  
  const int getZero(byte channel) {
    return transmitterZero[channel];
  }
  
  void setZero(byte channel, int value) {
    transmitterZero[channel] = value;
  }
  
  const int getTransmitterTrim(byte channel) {
    return transmitterTrim[channel];
  }
  
  void setTransmitterTrim(byte channel, int value) {
    transmitterTrim[channel] = value;
  }
  
  const float getSmoothFactor(byte channel) {
    return transmitterSmooth[channel];
  }
  
  void setSmoothFactor(byte channel, float value) {
    transmitterSmooth[channel] = value;
  }
  
  const float getXmitFactor(void) {
    return xmitFactor;
  }
  
  void setXmitFactor(float value) {
    xmitFactor = value;
  }
  
  const float getTransmitterSlope(byte channel) {
    return mTransmitter[channel];
  }
  
  void setTransmitterSlope(byte channel, float value) {
    mTransmitter[channel] = value;
  }
  
  const float getTransmitterOffset(byte channel) {
    return bTransmitter[channel];
  }
  
  void setTransmitterOffset(byte channel, float value) {
    bTransmitter[channel] = value;
  }
  
  const float getAngle(byte channel) {
    // Scale 1000-2000 usecs to -45 to 45 degrees
    // m = 0.09, b = -135
    // reduce transmitterCommand by xmitFactor to lower sensitivity of transmitter input
    return (0.09 * transmitterCommand[channel]) - 135;
  }
};



































/******************************************************/
/*************** AeroQuad Mega PCINT ******************/
/******************************************************/
#if defined(AeroQuadMega_v2)
  volatile uint8_t *port_to_pcmask[] = {
    &PCMSK0,
    &PCMSK1,
    &PCMSK2
  };
  volatile static uint8_t PCintLast[3];
  // Channel data 
  typedef struct {
    byte edge;
    unsigned long riseTime;    
    unsigned long fallTime; 
    unsigned long lastGoodWidth;
  } pinTimingData;  
  volatile static pinTimingData pinData[24]; 

  static void MegaPcIntISR() {
    uint8_t bit;
    uint8_t curr;
    uint8_t mask;
    uint8_t pin;
    uint32_t currentTime;
    uint32_t time;

    //curr = PORTK;
    curr = *portInputRegister(11);
    mask = curr ^ PCintLast[0];
    PCintLast[0] = curr;  

    //Serial.println(curr,DEC);

    // mask is pins that have changed. screen out non pcint pins.
    if ((mask &= PCMSK2) == 0) {
      return;
    }

    currentTime = micros();

    // mask is pcint pins that have changed.
    for (uint8_t i=0; i < 8; i++) {
      bit = 0x01 << i;
      if (bit & mask) {
        pin = i;
        // for each pin changed, record time of change
        if (bit & PCintLast[0]) {
         time = currentTime - pinData[pin].fallTime;
          pinData[pin].riseTime = currentTime;
          if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
            pinData[pin].edge = RISING_EDGE;
          else
            pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
        }
        else {
          time = currentTime - pinData[pin].riseTime;
          pinData[pin].fallTime = currentTime;
          if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
            pinData[pin].lastGoodWidth = time;
            //Serial.println(pinData[4].lastGoodWidth);
            pinData[pin].edge = FALLING_EDGE;
          } 
        }
      }
    }
  }

  SIGNAL(PCINT2_vect) {
    MegaPcIntISR();
  }
  
class Receiver_AeroQuadMega : public Receiver {
private:
  int receiverChannel[7];
  int receiverPin[7];
  //Receiver pin assignments for the Arduino Mega using an AeroQuad v1.x Shield
  //The defines below are for documentation only of the Mega receiver input
  //The real pin assignments happen in initializeMegaPcInt2()
  //If you are using an AQ 1.x Shield, put a jumper wire between the Shield and Mega as indicated in the comments below
  
  public:
  Receiver_AeroQuadMega() : Receiver(){}

  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x7F;//7f?!edit was 3F         //alter this to enable additional channel
    PCICR |= 0x1 << 2;
    

      receiverChannel[ROLL] = 63;
      receiverChannel[PITCH] = 64;
      receiverChannel[YAW] = 65;
      receiverChannel[THROTTLE] = 62;
      receiverChannel[MODE] = 66;
      receiverChannel[AUX] = 67;
      receiverChannel[AUX2] = 68;
      // defines ATmega328P pins (Arduino pins converted to ATmega328P pinouts)
      receiverPin[ROLL] = 1;
      receiverPin[PITCH] = 2;
      receiverPin[YAW] = 3;
      receiverPin[THROTTLE] = 0;
      receiverPin[MODE] = 4;
      receiverPin[AUX] = 5;
      receiverPin[AUX2] = 6;

    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      pinData[receiverChannel[channel]].edge = FALLING_EDGE;
  }
  
  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[7];
    uint8_t oldSREG;
      
    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    for (channel = ROLL; channel < LASTCHANNEL; channel++)
      data[channel] = pinData[receiverPin[channel]].lastGoodWidth;
    SREG = oldSREG;  
    
    for(channel = ROLL; channel < LASTCHANNEL; channel++) {
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs 
      transmitterCommandSmooth[channel] = smooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //transmitterCommandSmooth[channel] = transmitterFilter[channel].filter(receiverData[channel]);
    }
    // Reduce transmitter commands using xmitFactor and center around 1500
    for (channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

