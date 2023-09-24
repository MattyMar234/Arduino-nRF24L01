#include "NRF24L01_Interface.h"


NRF24L01_Interface::NRF24L01_Interface(uint8_t ce, uint8_t csn, SD_Card* SDcard)
{
    this->radio = new RF24(ce, csn);
    this->radioDeviceAvailable = false;
    this->SDcard = SDcard;

}

boolean NRF24L01_Interface::loadSettings() 
{
    File file;

    if(!this->SDcard->exists(NRF_FILE)) {
        if(GlobalVariable::debugportAvailable) {
            DebugPort.print(F("file "));
            DebugPort.print(NRF_FILE);
            DebugPort.println(F(" not found"));
        }
        return false;
    }

    file = this->SDcard->open(NRF_FILE, FILE_READ);

    if(!file) {
        if(GlobalVariable::debugportAvailable) {
            DebugPort.println(F("reading error"));
        }
        return false;
    }

    const static uint8_t size = 64;
    char buffer1[size];
    char buffer2[size];
    

    while (file.available()) 
    {
        uint8_t bufferIndex1 = 0;
        uint8_t bufferIndex2 = 0;
        
        if(!this->SDcard->parseStringUntill(&file, &buffer1[0], size , &bufferIndex1, '=')) {
            file.close();
            return false;
        }

        if(!this->SDcard->parseStringUntill(&file, &buffer2[0], size , &bufferIndex2, '\n')) {
            file.close();
            return false;
        }
        

        if(strcmp(buffer1, NRF_CHANNEL_KEY) == 0) {
            comunicationChannel = (uint8_t)atoi(buffer2);
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(NRF_CHANNEL_KEY); DebugPort.print(F(": ")); DebugPort.println(comunicationChannel);
            }
        }
        else if(strcmp(buffer1, NRF_AUTOACK_KEY) == 0) {
            autoAck = (strcmp(buffer2, "true") == 0 ? true : false);
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(NRF_AUTOACK_KEY);
                DebugPort.print(F(": "));
                DebugPort.println(autoAck);
            }
        }
        else if(strcmp(buffer1, NRF_RETRIES_DELAY_KEY) == 0) {
            retriesDelay = (uint8_t)atoi(buffer2);
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(NRF_RETRIES_DELAY_KEY); DebugPort.print(F(": ")); DebugPort.println(retriesDelay);
            }
        }
        else if(strcmp(buffer1, NRF_RETRIES_COUNT_KEY) == 0) {
            retriesCount = (uint8_t)atoi(buffer2);
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(NRF_RETRIES_COUNT_KEY); DebugPort.print(F(": ")); DebugPort.println(retriesCount);
            }
        }
        else if(strcmp(buffer1, NRF_DATA_RATE_KEY) == 0) {
            if(strcmp(buffer2, NRF_RATE1_KEY) == 0)
                dataRate = RF24_250KBPS;

            else if(strcmp(buffer2, NRF_RATE2_KEY) == 0)
                dataRate = RF24_1MBPS;
            
            else if(strcmp(buffer2, NRF_RATE3_KEY) == 0) 
                dataRate = RF24_2MBPS;
            
            else {
                if(GlobalVariable::debugportAvailable)
                    DebugPort.println("Invaid data rate");
                return false;
            }
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(NRF_DATA_RATE_KEY); DebugPort.print(F(": ")); DebugPort.println(dataRate);
            }
        }
        else if(strcmp(buffer1, NRF_POWER_LEVEL_KEY) == 0) {
            if(strcmp(buffer2, NRF_PA_LOW) == 0) {
                powerLevel = RF24_PA_LOW;
                if(GlobalVariable::debugportAvailable) 
                    DebugPort.print(NRF_DATA_RATE_KEY); DebugPort.print(F(": ")); DebugPort.println(F(NRF_PA_LOW));
            }
            else if(strcmp(buffer2, NRF_PA_HIGH) == 0) {
                powerLevel = RF24_PA_HIGH;
                if(GlobalVariable::debugportAvailable) 
                    DebugPort.print(NRF_DATA_RATE_KEY); DebugPort.print(F(": ")); DebugPort.println(F(NRF_PA_HIGH));
            }
            else {
                if(GlobalVariable::debugportAvailable)
                    DebugPort.println("Invaid data power level");
                return false;
            }
        }
        else if(strcmp(buffer1, NRF_READING_PIPE_KEY) == 0) {
            if(buffer2[0] != '0' || (buffer2[1] != 'x' && buffer2[1] != 'X')) {
                if(GlobalVariable::debugportAvailable) {
                    DebugPort.println(F("Invaid number for NRF_READING_PIPE_KEY (must be hex number)"));
                    return false;
                }
            }

            const char* inputHex = &buffer2[2];
            long res = this->SDcard->parseHexString(inputHex);

            if(res == -1) {
               if(GlobalVariable::debugportAvailable) {
                    DebugPort.println(F("hex number conversion error"));
                    return false;
                } 
            }
            ReadingPipe = (uint64_t)res;
        }
        else if(strcmp(buffer1, NRF_WRITING_PIPE_KEY) == 0) {
            if(buffer2[0] != '0' || (buffer2[1] != 'x' && buffer2[1] != 'X')) {
                if(GlobalVariable::debugportAvailable) {
                    DebugPort.println(F("Invaid number for WRITING_PIPE_KEY (must be hex number)"));
                    return false;
                }
            }

            const char* inputHex = &buffer2[2];
            long res = this->SDcard->parseHexString(inputHex);

            if(res == -1) {
               if(GlobalVariable::debugportAvailable) {
                    DebugPort.println(F("hex number conversion error"));
                    return false;
                } 
            }
            WritingPipe = (uint64_t)res;
        }
        else {
            if(GlobalVariable::debugportAvailable) {
                DebugPort.print(F("Invalid key: "));
                uint8_t i = 0;
                while(i < size && buffer1[i] != '\0') {
                    DebugPort.print(buffer1[i]); 
                }
                DebugPort.println();
            }
            return false;
        }    
    }
    return true;
}

void NRF24L01_Interface::reset() {
    if(!this->radioDeviceAvailable)
        return;

    radio->stopListening();

    for(uint8_t i = 0; i < 5; i++)
        radio->closeReadingPipe(i);

    setDefaoultSettings();
}

boolean NRF24L01_Interface::init()
{
    if(!this->radio->isChipConnected()) {
        if(GlobalVariable::debugportAvailable) DebugPort.println(F("NRF chip not found"));
        return false;
    }
    
    if(GlobalVariable::debugportAvailable) {
        DebugPort.println(F("NRF chip found."));
        DebugPort.println(F("reading data from SD card"));
    }

    
    if(SDcard == NULL || !SDcard->is_SD_Connected() || !loadSettings()) {
        if(GlobalVariable::debugportAvailable) DebugPort.println(F("SD card data not available. Loading defoult settings."));
        setDefaoultSettings();
    }
    
    if (!radio->begin()) {
        if(GlobalVariable::debugportAvailable)
            DebugPort.println(F("Inizializzation failed"));
        this->radioDeviceAvailable = false;
        return false;
    }
    

    this->radioDeviceAvailable = true;

    radio->setChannel(comunicationChannel);         //canale radio
    radio->setAutoAck(autoAck);                     //pacchetto di ricezione
    radio->enableDynamicPayloads();                 //pacchetto dinamico
    //radio->setPayloadSize(sizeof(payload));
    radio->setRetries(retriesDelay,retriesCount);   //tempo tra ritentativi, numero di tentativi
    radio->setDataRate(dataRate);                   //velocità di trasmissione
    radio->setPALevel(powerLevel);                  //potenza minima
    radio->powerUp();
    
    radio->openReadingPipe(0,ReadingPipe);       //device 1, canale di lettura 0
    radio->openReadingPipe(1,WritingPipe);       //device 2, canale di lettura 1

    radio->csDelay = 1;

    setWritingMode();
    
      
    if(GlobalVariable::debugportAvailable)
        DebugPort.println(F("Inizializzazione NRF24L01 completata\n"));

}

void NRF24L01_Interface::printDetails() {
    //this->radio->printDetails();
    this->radio->printPrettyDetails();
}




