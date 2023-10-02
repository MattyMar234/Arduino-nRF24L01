#pragma once

#include <MPU6050_tockn.h>
#include <Arduino.h>
#include "globalVariable.h"
#include "CardSD.h"
#include "Wire.h"


class MPU6050_Interface
{
    private:
        MPU6050 *mpu6050;
        boolean mpu6050Found; 
        SD_Card* cardSD;

        
        boolean loadSettings();
        boolean setDefaoultSettings();

    public:
        MPU6050_Interface(SD_Card* cardSD);
        boolean init();

        void dumpSettings();


        boolean inline available() { return mpu6050Found; }
};