#pragma once
#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H

#include <Arduino.h>

#define DebugPort Serial
#define Jnumber  6
#define SerialBufferLenght 32
#define NRF_CE 49
#define NRF_CSN 53
#define SD_CSN 47
#define ACTIVITY_LED 12
#define NRF_ErrorLED 11
#define SD_ErrorLED 10
#define MPU_ErrorLED 9

#define NRF_FILE "NRF24.txt"
#define NRF_CHANNEL_KEY "channel"
#define NRF_AUTOACK_KEY "autoAck"
#define NRF_RETRIES_DELAY_KEY "retriesDelay"
#define NRF_RETRIES_COUNT_KEY "retriesCount"
#define NRF_DATA_RATE_KEY "dataRate"
#define NRF_POWER_LEVEL_KEY "powerLevel"
#define NRF_READING_PIPE_KEY "readingPipe"
#define NRF_WRITING_PIPE_KEY "writingPipe"

#define NRF_RATE1_KEY  "250KBPS"
#define NRF_RATE2_KEY  "1MBPS"
#define NRF_RATE3_KEY  "2MBPS"
#define NRF_PA_LOW "PA_LOW"
#define NRF_PA_HIGH "PA_HIGH"


class GlobalVariable
{
    public:
        static boolean debugportAvailable;
};


//strutture
typedef struct TransmissionPacket_
{
  int16_t J1x;
  int16_t J2x;
  int16_t J3x;
  int16_t J4x;
  int16_t J5x;
  int16_t J6x;
  
  int16_t J1y;
  int16_t J2y;
  int16_t J3y;
  int16_t J4y;
  int16_t J5y;
  int16_t J6y;
  
  uint8_t  sw1;
  uint8_t  sw2;
  uint8_t  sw3;
  uint8_t  sw4;
  uint8_t  sw5;
  uint8_t  sw6;

  uint32_t Rx;
  uint32_t RY;
  uint32_t RZ;
  uint32_t Tm;
  
} TransmissionPacket;




#endif