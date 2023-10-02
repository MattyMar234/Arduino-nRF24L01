#include "MPU6050_Interface.h"


MPU6050_Interface::MPU6050_Interface(SD_Card* cardSD) {
    this->cardSD = cardSD;
}


boolean MPU6050_Interface::init() 
{
    uint8_t address;
    boolean found = false;

    Wire.begin();
    if(GlobalVariable::debugportAvailable)  {
        DebugPort.println(F("Scanning address 0x68 and 0x69 for MPU6050..."));
    }
        
    
    for(address = 0x68; address <= 0x69; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if(error == 0) {
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(F("I2C device found at address 0x"));
                DebugPort.println(address, HEX);
            } 
            found = true;
            break;
        }
    }

    if(!found) {
        if(GlobalVariable::debugportAvailable) {
            DebugPort.println(F("MPU6050 not found"));
        } 
    }
    
    mpu6050 = new MPU6050(Wire);
    //this->mpu6050Found = mpu6050.begin();
    

    //this->mpu6050Found = mpu6050.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);

    if(!this->mpu6050Found) {
        if(GlobalVariable::debugportAvailable) 
            DebugPort.println(F("MPU6050 initializzation failed"));
        return false;
    }
        

    if(this->cardSD == NULL || !cardSD->is_SD_Connected() || !loadSettings()) {
        if(GlobalVariable::debugportAvailable) DebugPort.println(F("SD card data not available. Loading defoult settings."));
            setDefaoultSettings();
    }

    //mpu6050.calcGyroOffsets(true);
    //mpu6050.calibrateGyro();
    //mpu6050.setThreshold(3);
}

boolean MPU6050_Interface::loadSettings()
{

    return true;
}


boolean MPU6050_Interface::setDefaoultSettings()
{
    return true;
}


void MPU6050_Interface::dumpSettings()
{
    /*DebugPort.print(F(" > Sleep Mode:        "));
    DebugPort.println(mpu6050.getSleepEnabled() ? F("Enabled") : F("Disabled"));
    
    DebugPort.print(F(" > Clock Source:      "));
    switch(mpu6050.getClockSource())
    {
        case MPU6050_CLOCK_KEEP_RESET:     DebugPort.println(F("Stops the clock and keeps the timing generator in reset")); break;
        case MPU6050_CLOCK_EXTERNAL_19MHZ: DebugPort.println(F("PLL with external 19.2MHz reference")); break;
        case MPU6050_CLOCK_EXTERNAL_32KHZ: DebugPort.println(F("PLL with external 32.768kHz reference")); break;
        case MPU6050_CLOCK_PLL_ZGYRO:      DebugPort.println(F("PLL with Z axis gyroscope reference")); break;
        case MPU6050_CLOCK_PLL_YGYRO:      DebugPort.println(F("PLL with Y axis gyroscope reference")); break;
        case MPU6050_CLOCK_PLL_XGYRO:      DebugPort.println(F("PLL with X axis gyroscope reference")); break;
        case MPU6050_CLOCK_INTERNAL_8MHZ:  DebugPort.println(F("Internal 8MHz oscillator")); break;
    }
    
    DebugPort.print(F(" > Gyroscope:         "));
    switch(mpu6050.getScale())
    {
        case MPU6050_SCALE_2000DPS:        DebugPort.println(F("2000 dps")); break;
        case MPU6050_SCALE_1000DPS:        DebugPort.println(F("1000 dps")); break;
        case MPU6050_SCALE_500DPS:         DebugPort.println(F("500 dps")); break;
        case MPU6050_SCALE_250DPS:         DebugPort.println(F("250 dps")); break;
    } 
    
    DebugPort.print(F(" > Gyroscope offsets: "));
    DebugPort.print(mpu6050.getGyroOffsetX());
    DebugPort.print(F(" / "));
    DebugPort.print(mpu6050.getGyroOffsetY());
    DebugPort.print(F(" / "));
    DebugPort.println(mpu6050.getGyroOffsetZ());
    DebugPort.println();*/
}