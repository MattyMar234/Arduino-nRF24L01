#include "CardSD.h"



SD_Card::SD_Card(uint8_t CSN_PIN_) {
    SD_CSN_PIN = CSN_PIN_;
    SD_card_detected = false;
}

boolean SD_Card::init() 
{
    if(!begin(this->SD_CSN_PIN)) {
        return false;
    }
    
    /*if (!card.init(SPI_HALF_SPEED, this->SD_CSN_PIN)) {
        this->SD_card_detected = false;
        return false;
    }


    if (!volume.init(card)) {
        InvalidPartition = true;
        return false;
    } */

    SD_card_detected = true;
    InvalidPartition = false;
    return true;
}




boolean SD_Card::parseStringUntill(File* file, char* buffer, uint8_t buffer_size, uint8_t* buffer_index, char end_char)
{
    while (file->available()) {
        char ch = file->read();

        //DebugPort.print(ch);

        if(ch == end_char) {
            buffer[(*buffer_index)++] = '\0';
            //DebugPort.println();
            return true;
        }

        //se ho saturato il buffer (n - 1 celle)
        if((*buffer_index) >= buffer_size - 1) {
            /*if(GlobalVariable::debugportAvailable)
                DebugPort.print(F("Error: keyword not found. Not enough data")); 
            DebugPort.println();*/
            return false;

        }

        if(ch > 32 && ch < 127)
            buffer[(*buffer_index)++] = ch;
    }

    /*if(GlobalVariable::debugportAvailable)
        DebugPort.print(F("error: keyword too long")); 
    DebugPort.println();*/
    return false;
}

long SD_Card::parseHexString(const char* str)
{
    long result = 0;
    int i = 0;

    while (str[i] != '\0') 
    {
        long digit = hexCharToLong(str[i]);
        if (digit == -1) {
            return -1;
        }
        result = result * 16 + digit;
        i++;
    }
    return result;
}


long SD_Card::hexCharToLong(char c) {
  if (isdigit(c)) {
    return c - '0';
  } else if (isxdigit(c)) {
    if (islower(c)) {
      return c - 'a' + 10;
    } else {
      return c - 'A' + 10;
    }
  } else {
    // Carattere non valido
    return -1;
  }
}





void SD_Card::dumpError() 
{
    if(!SD_card_detected) {
        DebugPort.println(F("SD initialization failed. Things to check:\n* is a card inserted?\n* is your wiring correct?\n* did you change the chipSelect pin to match your shield or module?"));
        return;
    }

    if(InvalidPartition) {
        DebugPort.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card with FAT16/FAT32"));
        return;
    }
}

void SD_Card::dumpSDInformation() 
{
    if(!SD_card_detected || InvalidPartition) {
        return;
    }


    //DebugPort.println(F("SD card found.\n"));
    DebugPort.println(F("Card specification:"));
    DebugPort.print(F("Card type:         "));
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:    DebugPort.println(F("SD1"));     break;
      case SD_CARD_TYPE_SD2:    DebugPort.println(F("SD2"));     break;
      case SD_CARD_TYPE_SDHC:   DebugPort.println(F("SDHC"));    break;
      default:
        DebugPort.println("Unknown");
    }

    // print the type and size of the first FAT-type volume
    DebugPort.print("Volume type is:    FAT"); 
    DebugPort.println(volume.fatType(), DEC);

    DebugPort.print(F("Clusters:          "));
    DebugPort.println(volume.clusterCount());
    DebugPort.print(F("Blocks x Cluster:  "));
    DebugPort.println(volume.blocksPerCluster());
    DebugPort.print(F("Total Blocks:      "));
    DebugPort.println(volume.blocksPerCluster() * volume.clusterCount());
    //DebugPort.println();

    uint32_t volumesize;
    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    DebugPort.print(F("Volume size (Kb):  "));
    DebugPort.println(volumesize);
    DebugPort.print(F("Volume size (Mb):  "));
    volumesize /= 1024;
    DebugPort.println(volumesize);
    DebugPort.print(F("Volume size (Gb):  "));
    DebugPort.println((float)volumesize / 1024.0);


    DebugPort.println(F("\nFiles found on the card: "));
    root.openRoot(volume);

    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
}