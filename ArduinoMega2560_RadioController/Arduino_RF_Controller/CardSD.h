#pragma once
#ifndef CARDSD_H
#define CARDSD_H

#include <SD.h>
#include "globalVariable.h"

using namespace SDLib;



class SD_Card : public SDClass
{
    private:
        uint8_t SD_CSN_PIN;
        /*Sd2Card card;
        SdVolume volume;
        SdFile root;*/
        //SDClass sd;

        boolean SD_card_detected;
        boolean InvalidPartition;
  
    public:
        SD_Card(uint8_t CSN_PIN);
        boolean init(); 
        boolean is_SD_Connected() { return card.cardSize() != 0; }
        void dumpSDInformation();
        void dumpError();

        boolean parseStringUntill(File* file,char* buffer, uint8_t buffer_size, uint8_t* buffer_index, char end_char);
        long hexCharToLong(char c);
        long parseHexString(const char* str);

};

//extern SD_Card SD_Card;

#endif